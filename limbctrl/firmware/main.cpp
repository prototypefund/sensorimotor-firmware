#include <xpcc/architecture/platform.hpp>

#include <src/communication.hpp>

using namespace Board;

using GlobalSync = xpcc::stm32::Timer2;
using LocalDelay = xpcc::stm32::Timer3;
using RxTimeout  = xpcc::stm32::Timer4;

enum CycleState {
	initializing   = 0,
	synchronizing  = 1,
	synchronized   = 2,
	receiving      = 3,
	transmitting   = 4,
	doublicate_id  = 5,
};

enum RecvState {
	sync0     = 0,
	sync1     = 1,
	read_id   = 2,
	read_data = 3,
	validate  = 4,
	success   = 5,
	error     = 6,
	done      = 7,
};


/* 1Mbaud/s = 1.000.000 baud/s = 100.000 byte/s
   with 10 bit per byte (8N1)
   -> 10 us per byte
*/
constexpr unsigned byte_transmission_time_us = 10;
constexpr unsigned deadtime_us = 10;
constexpr unsigned bytes_per_slot = 10;
constexpr unsigned slottime_us = bytes_per_slot * byte_transmission_time_us + deadtime_us;

constexpr uint8_t syncbyte = 0x55;
constexpr uint8_t board_id = 3;    //TODO read from EEPROM
constexpr uint8_t max_id = 8;
constexpr unsigned frametime_us = 10000; /* 10.000 us = 10 ms = 100 Hz */
constexpr unsigned local_delay_us = board_id * slottime_us;

static_assert(slottime_us > 0);
static_assert(local_delay_us > 0);

uint8_t packets = 0;
uint8_t errors  = 0;
uint8_t cycles = 0;

uint8_t send_buffer[bytes_per_slot];

/*
	5 boards à 400 us = 2000 us = 2 ms communication time
	3 motors à x us = y us ???

	/// cycle partition ///
	0.0 communication
	2.0 start neural calculations
	8.0 update motors
	    + write motors
	    + read motors
	    + read local sensors
	    + trigger ADC for local sensors
*/

/* variables used by timer ISRs */
volatile CycleState state = initializing;
volatile bool rx_timed_out = false;
volatile bool sendnow      = false;
volatile bool syncnow      = false;

// TODO using TrayType = uint16_t;

bool read_slot(uint8_t& received_id) {
	static RecvState rx_state = sync0;
	static uint8_t data = 0;
	static uint8_t bytecount = 0;
	static uint8_t checksum = 0;

	bool result = false;

	if (rx_timed_out) {
		rx_state = sync0;
		++errors;
		rx_timed_out = false;
	}

	switch(rx_state)
	{
		case sync0:
			if (spinalcord::uart::read(data)) {
				if (data == syncbyte) {
					rx_state = sync1;
					checksum = syncbyte;
				}
				// stay in sync0 otherwise
			}
			break;

		case sync1:
			if (spinalcord::uart::read(data)) {
				if (data == syncbyte) {
					rx_state = read_id;
					checksum += syncbyte;
					reset_and_start_timer<RxTimeout>();
				}
				else
					rx_state = sync0;
			}
			break;

		case read_id:
			if (spinalcord::uart::read(data)) {
				if (data < max_id) {
					received_id = data;
					checksum += received_id;
					rx_state = read_data;
					bytecount = 0;
				} else
					rx_state = error; // did not received a valid id byte
			}
			break;

		case read_data:
			if (spinalcord::uart::read(data)) {
				++bytecount;
				checksum += data;
				if (bytecount == bytes_per_slot - 3/*preamble*/)
					rx_state = validate;
			}
			break;

		case validate:
			rx_state = (checksum == 0) ? success : error;
			break;

		case success: result = true;  rx_state = done; ++packets; break;
		case error:   result = false; rx_state = done; ++errors;  break;

		case done:
		default:
			RxTimeout::pause(); // stop timer before leaving
			rx_state = sync0;      // start over again
			break;

	} // switch(rx_state)

	return result; // return true of valid slot could be read
}

void init_send_buffer(void)
{
	/* clear buffer */
	memset(send_buffer, 0, bytes_per_slot);
	/* add preamble */
	send_buffer[0] = syncbyte;
	send_buffer[1] = syncbyte;
	send_buffer[2] = board_id;
}

//TODO unify with sendbuffer from sensorimotor
void prepare_transmission(uint8_t min_id, uint8_t last_min_id, uint8_t board_list)
{
	send_buffer[3] = min_id;
	send_buffer[4] = last_min_id;
	send_buffer[5] = board_list;
	send_buffer[6] = packets;
	send_buffer[7] = errors;
	send_buffer[8] = cycles;

	/* create checksum */
	uint8_t checksum = 0;
	for (unsigned i = 0; i < bytes_per_slot-1; ++i)
		checksum += send_buffer[i];
	send_buffer[bytes_per_slot-1] = ~checksum + 1;
}

void send_data(void) {
	spinalcord_send_mode();
	spinalcord::uart::write(send_buffer, bytes_per_slot);
	spinalcord::uart::flushWriteBuffer();
	spinalcord_receive_mode();
}

void error_state(void)
{
	led_red::set();
	led_ylw::reset();
	while(1) {
		led_red::toggle();
		led_ylw::toggle();
		xpcc::delayMilliseconds(500);
	}
}

int
main()
{
	Board::initialize();

	motorcord_receive_mode();
	spinalcord_receive_mode();

	init_timer<GlobalSync, frametime_us  >();
	init_timer<LocalDelay, local_delay_us>();
	init_timer<RxTimeout , slottime_us   >();

	uint8_t min_id = board_id; // assume, until we know better
	uint8_t last_min_id = 255;

	uint8_t slot_id = 255;
	unsigned synctime_us = frametime_us;
	uint8_t board_list = 0;
	uint8_t last_board_list = 0;

	bool timer_started = false;

	init_send_buffer();

	while (1)
	{
		switch(state)
		{
		case initializing: /* start first frame and try to (re-)sync */
			xpcc::delayMilliseconds(board_id*2);
			GlobalSync::start();
			state = synchronizing;
			min_id = board_id;//max_id;
			led_red::set();
			break;

		case synchronizing: /* searching for minimal id */
			if (read_slot(slot_id))
			{
				if (slot_id < min_id) { // found new board
					min_id = slot_id;
					reset_and_start_timer<GlobalSync>();
				}

				if (slot_id == board_id) // someone is using our id!
					state = doublicate_id;
			}
			// note: state is set to [synchronized] by global sync ISR
			if (syncnow) {
				state = synchronized;
				syncnow = false;
			}
			break;

		case synchronized:
			++cycles;
			if (last_min_id != min_id) { // we got a new min_id, timer must by changed
				synctime_us = frametime_us - min_id * slottime_us;
				GlobalSync::setPeriod<Board::systemClock>(synctime_us, /*autoapply=*/ true);
				last_min_id = min_id; // remember last min_id before resetting
				if (last_min_id == board_id) // we are leading
					led_ylw::set();
				else
					led_ylw::reset();
			}
			state = (board_id == 0) ? transmitting : receiving;
			min_id = board_id; // reset
			last_board_list = board_list;
			board_list = 1 << board_id;
			timer_started = false;
			break;

		case receiving:
			if (read_slot(slot_id))
			{
				//TODO check with expected boards, from last_board_list, resync on deviation

				board_list |= 1 << slot_id;

				if (!timer_started and slot_id >= last_min_id) // we found the leading board
				{
					reset_and_start_timer<GlobalSync>();
					timer_started = true;
				}

				if (slot_id < min_id)
					min_id = slot_id;

				if (slot_id == board_id)
					state = doublicate_id;

				//put data to spinalcord array here
			}
			if (sendnow)
				state = transmitting;
			if (syncnow) {
				state = synchronized;
				syncnow = false;
			}
			break;

		case transmitting:
			prepare_transmission(min_id, last_min_id, last_board_list);
			send_data();
			if (min_id == board_id) { // it seems that we are the leading board
				reset_and_start_timer<GlobalSync>();
				timer_started = true;
			}
			state = receiving;
			sendnow = false;
			break;

		case doublicate_id: error_state(); break;

		} // switch(state)

	} // while(1)
	return 0;
}

/*
	/// synchronizing procedure ///

	min_id = board_id // we must assume, that we are the first board, until we know better.

	re-sync if min_id has changed:
		setperiod ( frametime_us - (min_id+1) * slottime_us)
		auto-apply = true
*/

XPCC_ISR(TIM2)
{
	LocalDelay::start();
	GlobalSync::acknowledgeInterruptFlags(GlobalSync::InterruptFlag::Update);
	GlobalSync::pause();
	led_red::toggle();
	syncnow = true;
}

XPCC_ISR(TIM3)
{
	LocalDelay::acknowledgeInterruptFlags(LocalDelay::InterruptFlag::Update);
	LocalDelay::pause();
	sendnow = true; // trigger sending own data now
}

XPCC_ISR(TIM4)
{
	RxTimeout::acknowledgeInterruptFlags(RxTimeout::InterruptFlag::Update);
	RxTimeout::pause();
	rx_timed_out = true;
}
