/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#include <xpcc/architecture/platform.hpp>
#include <boards/limbctrl_f411re.hpp>
#include <src/common.hpp>
#include <src/timer.hpp>
#include <src/spinalcord.hpp>
#include <src/communication.hpp>

#include <src/motorcord.hpp>

using namespace Board;
using namespace supreme;

using GlobalSync = xpcc::stm32::Timer2;
using LocalDelay = xpcc::stm32::Timer3;
using RxTimeout  = xpcc::stm32::Timer4;
/* Tim5 is currently used for both, signalling motor slot and motor rx timeout */
using MotorTimer = xpcc::stm32::Timer5;

enum CycleState {
	initializing   = 0,
	synchronizing,
	synchronized,
	receiving,
	transmitting,
	receiving_2,
	writing_motors,
	idle,
	duplicate_id,
};

/* 3 Mbaud/s = 3.000.000 baud/s = 300.000 byte/s
   with 10 bit per byte (8N1)
   -> 3.34 us per byte
*/
constexpr uint8_t board_id = 1;    //TODO read from EEPROM

constexpr unsigned byte_transmission_time_us = 10; //TODO
constexpr unsigned deadtime_us = 20;
constexpr unsigned bytes_per_slot = 64;
constexpr unsigned slottime_us = bytes_per_slot * byte_transmission_time_us + deadtime_us;

constexpr uint8_t trunk_id = 4;
constexpr uint8_t syncbyte = 0x55;
constexpr uint8_t max_id = 7;
constexpr unsigned frametime_us = 10000; /* 10.000 us = 10 ms = 100 Hz */
constexpr unsigned local_delay_us = board_id * slottime_us + deadtime_us;

constexpr unsigned motortime_us = frametime_us - 2000;

constexpr bool is_trunk_controller = (board_id == trunk_id);

// constexpr unsigned full_size = bytes_per_slot * (max_id+1);

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

void signal_leading(uint8_t ref_id) {
	if (ref_id == board_id) // we are leading
		led_ylw::set();
	else
		led_ylw::reset();
}


/* variables used by timer ISRs */
volatile bool rx_timed_out = false;
volatile bool sendnow      = false;
volatile bool syncnow      = false;
volatile bool write_motors = false;


void apply_global_sync(uint8_t ref_id) {
	unsigned synctime_us = frametime_us - ref_id * slottime_us;
	setperiod_and_restart_timer<GlobalSync>(synctime_us);
}

bool we_are(uint8_t ref_id) { return board_id == ref_id; }

int
main()
{
	Board::initialize();

	init_timer<GlobalSync, frametime_us*10>();
	init_timer<LocalDelay, local_delay_us>();
	init_timer<MotorTimer, motortime_us  >();

	CycleState state = initializing;

	uint8_t leading_id = board_id; // assume, until we know better
	uint8_t transparent_mode = 0;
	uint8_t board_list = 0;
	uint8_t last_board_list = 0;

	bool timer_started = false;

	/* carrier for voltage setpoints, TODO integrate in SC-Data structure */
	typedef supreme::MotorCord<rs485_motorcord, MotorTimer, board_id, 3> MotorCord_t;
	typedef supreme::SpinalCord<rs485_spinalcord, bytes_per_slot, syncbyte, MotorCord_t, board_id> SpinalCord_t;

	MotorCord_t::target_voltage_t target_voltages;
	target_voltages.fill(float_to_sc(0.0));

	MotorCord_t motorcord(target_voltages);
	SpinalCord_t spinalcord(motorcord);

	supreme::CommunicationController<RxTimeout, syncbyte, max_id, bytes_per_slot, slottime_us, MotorCord_t::target_voltage_t> com(&rx_timed_out, target_voltages);

	SpinalCordFull<rs485_external> sc_full;

	typedef TransparentData<rs485_external, rs485_spinalcord> TransparentData_t;
	TransparentData_t transparent_data;

	uint8_t cycles = 0;


	while (1)
	{

		/* other stuff */
		sc_full.check_transmission_finished();

		switch(state)
		{
		case initializing: /* start first frame and try to (re-)sync */
			state = synchronizing;
			GlobalSync::pause();
			setperiod_and_restart_timer<GlobalSync>(10*frametime_us);
			leading_id = board_id;
			led_red::reset();
			led_ylw::reset();
			break;

		case synchronizing: /* searching for minimal id */
			led_ylw::set();
			if (com.read_slot())
			{
				led_red::set();
				uint8_t slot_id = com.get_received_id();

				/* sync to the first board you can find */
				apply_global_sync(slot_id);
				leading_id = slot_id;
				while (!syncnow) {} // note: syncnow is set by GlobalSync ISR

				syncnow = false;
				state = synchronized;
			}
			else {
				if (syncnow) { // tried enough finding a leading board
					syncnow = false;
					apply_global_sync(board_id);
					state = synchronized;
				};
			}

			break;

		case synchronized:
			led_red::set();
			led_ylw::reset();
			signal_leading(leading_id);
			++cycles;

			state = receiving;
			leading_id = board_id; // reset
			last_board_list = board_list;
			board_list = 1 << board_id;
			timer_started = false;
			break;

		case receiving:
			if (com.read_slot())
			{
				uint8_t slot_id = com.get_received_id();
				board_list |= 1 << slot_id;

				if (slot_id < leading_id)
					leading_id = slot_id;

				/* we found the leading board */
				if (!we_are(leading_id) and !timer_started and slot_id == leading_id)
				{
					apply_global_sync(leading_id);
					timer_started = true;
				}

				if (slot_id == board_id) // someone is using our id!
					state = duplicate_id;

				if (is_trunk_controller)
					sc_full.start_transmission(com.get());
			}

			if (sendnow) {
				sendnow = false;
				state = transmitting;
			}

			break;

		case transmitting:
			led_red::reset();
			spinalcord.prepare( leading_id, transparent_mode, last_board_list,
			                     com.packets, com.errors, cycles );
			spinalcord.transmit();
			if (we_are(leading_id)) { // we are the leading board must sync to ourselves
				apply_global_sync(board_id);
			 	timer_started = true;
			}

			if (is_trunk_controller)
				sc_full.start_transmission(spinalcord.get());

			state = receiving_2;
			break;

		case receiving_2:

			if (!timer_started) { // something went wrong
				state = initializing;
			}

			if (com.read_slot())
			{
				uint8_t slot_id = com.get_received_id();
				board_list |= 1 << slot_id;

				if (is_trunk_controller)
					sc_full.start_transmission(com.get());
			}

			if (is_trunk_controller && transparent_data.read()) {
				transparent_data.write();
				++transparent_mode;
			}

			if (write_motors) {
				state = writing_motors;
				write_motors = false;
				motorcord.prepare();
			}
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

	//TODO remove reset here, to check for timed out from elsewhere, just pause... and get rid of the additional bools, only pause the timer here. do not ackknowledge here. do the ack in the "is_timed_out()" method
}

XPCC_ISR(TIM5)
{
	MotorTimer::acknowledgeInterruptFlags(MotorTimer::InterruptFlag::Update);
	MotorTimer::applyAndReset();
	MotorTimer::pause();
	write_motors = true;
}
