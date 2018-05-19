#include <xpcc/architecture/platform.hpp>

#include <src/common.hpp>
#include <src/timer.hpp>
#include <src/spinalcord.hpp>
#include <src/communication.hpp>

#include <src/ux_com.hpp>

using namespace Board;
using namespace supreme;

using GlobalSync = xpcc::stm32::Timer2;
using LocalDelay = xpcc::stm32::Timer3;
using RxTimeout  = xpcc::stm32::Timer4;
/* Tim5 is currently used for both, signalling motor slot and motor rx timeout */
using MotorTimer = xpcc::stm32::Timer5;

enum CycleState {
	initializing   = 0,
	synchronizing  = 1,
	synchronized   = 2,
	receiving      = 3,
	transmitting   = 4,
	writing_motors = 5,
	idle           = 6,
	duplicate_id   = 7,
};

/* 3 Mbaud/s = 3.000.000 baud/s = 300.000 byte/s
   with 10 bit per byte (8N1)
   -> 3.34 us per byte

	/// synchronizing procedure ///
	min_id = board_id // we must assume, that we are the first board, until we know better.
	re-sync if min_id has changed:
		setperiod ( frametime_us - min_id * slottime_us)
*/

constexpr unsigned byte_transmission_time_us = 10; //TODO
constexpr unsigned deadtime_us = 20;
constexpr unsigned bytes_per_slot = 64;
constexpr unsigned slottime_us = bytes_per_slot * byte_transmission_time_us + deadtime_us;

constexpr uint8_t syncbyte = 0x55;
constexpr uint8_t board_id = 1;    //TODO read from EEPROM
constexpr uint8_t max_id = 8;
constexpr unsigned frametime_us = 10000; /* 10.000 us = 10 ms = 100 Hz */
constexpr unsigned local_delay_us = board_id * slottime_us;

constexpr unsigned motortime_us = frametime_us - 2000;

static_assert(slottime_us > 0);
static_assert(local_delay_us > 0);

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
volatile bool rx_timed_out = false;
volatile bool sendnow      = false;
volatile bool syncnow      = false;
volatile bool write_motors = false;


int
main()
{
	Board::initialize();

	init_timer<GlobalSync, frametime_us*2>();
	init_timer<LocalDelay, local_delay_us>();
	init_timer<RxTimeout , slottime_us   >(); //TODO move to communication
	init_timer<MotorTimer, motortime_us  >();

	CycleState state = initializing;

	uint8_t min_id = board_id; // assume, until we know better
	uint8_t last_min_id = 255;

	unsigned synctime_us = frametime_us;
	uint8_t board_list = 0;
	uint8_t last_board_list = 0;

	bool timer_started = false;


	/* carrier for voltage setpoints, TODO integrate in SC-Data structure */
	typedef supreme::MotorCord<rs485_motorcord, MotorTimer, board_id, 3> MotorCord_t;
	typedef supreme::SpinalCord<rs485_spinalcord, bytes_per_slot, syncbyte, MotorCord_t, board_id> SpinalCord_t;

	MotorCord_t::target_voltage_t target_voltages;
	target_voltages.fill(float_to_sc(0.1));

	MotorCord_t motorcord(target_voltages);
	SpinalCord_t spinalcord(motorcord);

	supreme::CommunicationController<RxTimeout, syncbyte, max_id, bytes_per_slot> com;

	uint8_t cycles = 0;

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
			if (com.read_slot(rx_timed_out))
			{
				uint8_t slot_id = com.get_received_id();
				if (slot_id < min_id) { // found new board
					min_id = slot_id;
					reset_and_start_timer<GlobalSync>();
				}

				if (slot_id == board_id) // someone is using our id!
					state = duplicate_id;
			}
			// note: state is set to [synchronized] by global sync ISR
			if (syncnow) {
				state = synchronized;
				syncnow = false;
			}
			break;

		case synchronized:
			++cycles;
			if (min_id < last_min_id) {
				// we got a new min_id, timer must by changed
				synctime_us = frametime_us - min_id * slottime_us;
				GlobalSync::setPeriod<Board::systemClock>(synctime_us, /*autoapply=*/ true);
				last_min_id = min_id; // remember last min_id before resetting
				if (last_min_id == board_id) // we are leading
					led_ylw::set();
				else
					led_ylw::reset();
			}
			else if (min_id > last_min_id) { // lost master, trigger re-sync!
				state = initializing;
				break;
			}
			state = (board_id == 0) ? transmitting : receiving;
			min_id = board_id; // reset
			last_board_list = board_list;
			board_list = 1 << board_id;
			timer_started = false;
			break;

		case receiving:
			if (com.read_slot(rx_timed_out))
			{
				uint8_t slot_id = com.get_received_id();
				//TODO check with expected boards, from last_board_list, resync on deviation

				board_list |= 1 << slot_id;

				if (!timer_started and slot_id >= last_min_id) // we found the former leading board
				{
					reset_and_start_timer<GlobalSync>();
					timer_started = true;
				}

				if (slot_id < min_id)
					min_id = slot_id;

				if (slot_id == board_id)
					state = duplicate_id;

				//put data to spinalcord array here
			}
			if (sendnow)
				state = transmitting;

			if (write_motors) {
				state = writing_motors;
				write_motors = false;
				motorcord.prepare();
			}
			break;

		case transmitting:
			spinalcord.prepare( min_id, last_min_id, last_board_list,
			                     com.packets, com.errors, cycles );
			spinalcord.transmit();
			if (min_id == board_id) { // it seems that we are the leading board
				reset_and_start_timer<GlobalSync>();
				timer_started = true;
			}
			state = receiving;
			sendnow = false;
			break;

		case writing_motors:
			switch( motorcord.transmit(write_motors) )
			{
			case MotorCord_t::State_t::done:
				state = idle;
				/*Fall through*/
			case MotorCord_t::State_t::waiting_for_next:
				write_motors = false;
				break;
			default: break;
			} // switch
			break;

		case idle:
			if (syncnow) {
				state = synchronized;
				syncnow = false;
			}
			break;

		case duplicate_id: error_state(); break;

		} // switch(state)

	} // while(1)
	return 0;
}


/* Timer Interrupt Service Routines */
XPCC_ISR(TIM2)
{
	LocalDelay::start();
	MotorTimer::setPeriod<Board::systemClock>(motortime_us);
	MotorTimer::start();
	GlobalSync::acknowledgeInterruptFlags(GlobalSync::InterruptFlag::Update);
	GlobalSync::applyAndReset();
	GlobalSync::pause();
	led_red::toggle();
	syncnow = true;
}

XPCC_ISR(TIM3)
{
	LocalDelay::acknowledgeInterruptFlags(LocalDelay::InterruptFlag::Update);
	LocalDelay::applyAndReset();
	LocalDelay::pause();
	sendnow = true; // trigger sending own data now
}

XPCC_ISR(TIM4)
{
	RxTimeout::acknowledgeInterruptFlags(RxTimeout::InterruptFlag::Update);
	RxTimeout::applyAndReset();
	RxTimeout::pause();
	rx_timed_out = true;
}

XPCC_ISR(TIM5)
{
	MotorTimer::acknowledgeInterruptFlags(MotorTimer::InterruptFlag::Update);
	MotorTimer::applyAndReset();
	MotorTimer::pause();
	write_motors = true;
}
