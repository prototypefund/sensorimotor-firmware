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
	success   = 4,
	error     = 5,
	done      = 6,
};


/* 1Mbaud/s = 1.000.000 baud/s = 100.000 byte/s
   with 10 bit per byte (8N1)
   -> 10 us per byte
*/
constexpr unsigned byte_transmission_time_us = 10;
constexpr unsigned deadtime_us = 80;
constexpr unsigned bytes_per_slot = 32;
constexpr unsigned slottime_us = bytes_per_slot * byte_transmission_time_us
                               + deadtime_us; // 32*10 + 80 = 400

constexpr uint8_t syncbyte = 0x55;
constexpr uint8_t board_id = 3;    //TODO read from EEPROM
constexpr uint8_t max_id = 8;
constexpr unsigned frametime_us = 500000; /* 10.000 us = 10 ms = 100 Hz */
constexpr unsigned local_delay_us = board_id * slottime_us;

static_assert(slottime_us > 0);
static_assert(local_delay_us > 0);

uint8_t packets = 0;

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


bool read_slot(uint8_t& received_id) {
	static RecvState rx_state = sync0;
	static uint8_t data = 0;
	static uint8_t bytecount = 0;
	bool result = false;

	if (rx_timed_out)
		rx_state = sync0;

	switch(rx_state)
	{
		case sync0:
			if (spinalcord::uart::read(data)) {
				if (data == syncbyte) {
					rx_state = sync1;
				}
				// stay in sync0 otherwise
			}
			break;

		case sync1:
			if (spinalcord::uart::read(data)) {
				if (data == syncbyte) {
					rx_state = read_id;
					rx_timed_out = false;
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
					rx_state = read_data;
					bytecount = 0;
				} else
					rx_state = error; // did not received a valid id byte
			}
			break;

		case read_data:
			if (spinalcord::uart::read(data)) {
				++bytecount;
				if (bytecount == 3)
					rx_state = success;
			}
			break;

		case success: result = true;  rx_state = done; packets++; break;
		case error:   result = false; rx_state = done; break;

		case done:
		default:

			RxTimeout::pause(); // stop timer before leaving
			rx_state = sync0;      // start over again
			break;

	} // switch(rx_state)

	return result; // return true of valid slot could be read
}

void send_data(uint8_t min_id, uint8_t last_min_id) {
	const unsigned N = 6;

	uint8_t buffer[N];
	memset(buffer, 0, N); // clear buffer

	buffer[0] = syncbyte;
	buffer[1] = syncbyte;
	buffer[2] = board_id;

	buffer[3] = min_id;
	buffer[4] = last_min_id;
	buffer[5] = packets;

	spinalcord_send_mode();
	spinalcord::uart::write(buffer, N);
	spinalcord::uart::flushWriteBuffer();
	spinalcord_receive_mode();
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
			break;

		case synchronized:
			if (last_min_id != min_id) { // we got a new min_id, timer must by changed
				synctime_us = frametime_us - (min_id + 1) * slottime_us;
				GlobalSync::setPeriod<Board::systemClock>(synctime_us, /*autoapply=*/ true);
				last_min_id = min_id; // remember last min_id before resetting
				if (last_min_id == board_id) // we are leading
					led_ylw::set();
				else
					led_ylw::reset();
			}
			state = (board_id == 0) ? transmitting : receiving;
			min_id = board_id; // reset

			break;

		case receiving:
			if (read_slot(slot_id))
			{
				if (slot_id == last_min_id) // we found the leading board
				{
					reset_and_start_timer<GlobalSync>();
				}

				if (slot_id < min_id)
					min_id = slot_id;

				if (slot_id == board_id)
					state = doublicate_id;

				//put data to spinalcord array here
			}
			break;

		case transmitting:
			send_data(min_id, last_min_id);
			state = receiving;
			if (min_id == board_id) {// it seems that we are leading
				reset_and_start_timer<GlobalSync>();
			}
			break;

		case doublicate_id:
			led_red::set();
			led_ylw::set();
			xpcc::delayMilliseconds(3000); // wait a second before next trial
			led_red::reset();
			led_ylw::reset();
			state = initializing;
			break;

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
	GlobalSync::acknowledgeInterruptFlags(GlobalSync::InterruptFlag::Update);
	GlobalSync::pause();
	led_red::toggle();
	state = synchronized;
	LocalDelay::start();
}

XPCC_ISR(TIM3)
{
	LocalDelay::acknowledgeInterruptFlags(LocalDelay::InterruptFlag::Update);
	LocalDelay::pause();
	state = transmitting; // send own data now
}

XPCC_ISR(TIM4)
{
	RxTimeout::acknowledgeInterruptFlags(RxTimeout::InterruptFlag::Update);
	RxTimeout::pause();
	rx_timed_out = true;
}
