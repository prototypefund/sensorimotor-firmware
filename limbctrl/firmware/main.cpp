#include <xpcc/architecture/platform.hpp>

#include <src/common.hpp>
#include <src/timer.hpp>
#include <src/sendbuffer.hpp>
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
constexpr uint8_t board_id = 5;    //TODO read from EEPROM
constexpr uint8_t max_id = 8;
constexpr unsigned frametime_us = 10000; /* 10.000 us = 10 ms = 100 Hz */
constexpr unsigned local_delay_us = board_id * slottime_us;

constexpr unsigned motortime_us = frametime_us - 2000;

/*
const uint8_t motors_per_board[4][3] = { { 0,  2,  4} // 0
                                       , { 1,  3,  5} // i/2 * 6 + i%2 + 2*j
                                       , { 6,  8, 10} // 2/2 * 6 + 2%2
                                       , { 7,  9, 11} // 3/2 * 6 + 3%2
                                       , {12, 14, 16} // 4
                                       }; */
constexpr
uint8_t get_motor_id_from_board_id(uint8_t board_id, uint8_t motor_index) {
	return board_id/2 * 6 + board_id%2 + 2*motor_index;
}

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

	supreme::Sendbuffer<rs485_spinalcord, bytes_per_slot, syncbyte> send_buffer(board_id);

	supreme::CommunicationController<RxTimeout, syncbyte, max_id, bytes_per_slot> com;

	constexpr uint8_t motor_ids[3] = { get_motor_id_from_board_id(board_id, 0)
	                                 , get_motor_id_from_board_id(board_id, 1)
	                                 , get_motor_id_from_board_id(board_id, 2) };


	// TODO ping motors...and check if motors are connected correctly.

	uint8_t cycles = 0;


	unsigned motor_idx = 0;
	supreme::ux_communication_ctrl<rs485_motorcord, MotorTimer> motor_1(1);
	supreme::ux_communication_ctrl<rs485_motorcord, MotorTimer> motor_2(2);
	supreme::ux_communication_ctrl<rs485_motorcord, MotorTimer> motor_3(3);

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
			}
			break;

		case transmitting:
			send_buffer.prepare( min_id, last_min_id, last_board_list,
			                     com.packets, com.errors, cycles );
			send_buffer.transmit();
			if (min_id == board_id) { // it seems that we are the leading board
				reset_and_start_timer<GlobalSync>();
				timer_started = true;
			}
			state = receiving;
			sendnow = false;
			break;

		case writing_motors:
			switch(motor_idx) {
			case 0:
				if (motor_3.step(write_motors)) {
					write_motors = false;
					++motor_idx;
				}
				break;
			case 1:
				if (motor_2.step(write_motors)) {
					write_motors = false;
					++motor_idx;
				}
				break;
			case 2:
				if (motor_1.step(write_motors)) {
					write_motors = false;
					state = idle;
					motor_idx = 0;
				}
				break;
			}

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
