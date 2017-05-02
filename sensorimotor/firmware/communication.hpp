#include <xpcc/architecture/platform.hpp>

namespace supreme {

	inline void helloworld() {
		/* send hello */
		Uart0::write('u');
		Uart0::write('x');
		Uart0::write('0');
	}


class communication_ctrl {

	/*enum command_t {
		no_command     = 0,
		data_requested = 1,
		toggle_enable  = 2,
		set_voltage    = 3,
		toggle_led     = 4,
		increase_pwm   = 5,
		decrease_pwm   = 6,
		release_mode   = 7,
	};*/

	// use enum type : 0recv, 1waiting for data, 2accomplished
	bool data_requested = false;
	bool toggle_enable  = false;
	unsigned set_voltage = 0;
	bool toggle_led     = false;
	bool increase_pwm   = false;
	bool decrease_pwm   = false;
	bool release_mode   = false;

	supreme::sensorimotor_core& ux;
	uint8_t buffer;
	uint16_t position;
	bool     direction;

	const uint8_t motor_id = 17;

	//command_t command;

	/**
		enum command id
		bytes received
		command state
			0 wait
			1 reading
			2 data corrupt? possible command interrupt
			3 finished


	*/


	bool led_state = false;
	uint8_t target_pwm = 0;

public:

	communication_ctrl(supreme::sensorimotor_core& ux)
	: ux(ux)
	, position()
	, direction()
	//, command(no_command)
	{}


	bool search_for_command() {

		if (set_voltage==2)
			set_voltage=0;

		if (set_voltage==1)
			set_voltage=2;


		switch(buffer) {
			case 0xC0: /* 1100.0000 */ data_requested = true;   return true;
			case 0xA0: /* 1010.0000 */ toggle_enable  = true;   return true;

			case 0xB0: /* 1011.0000 */ //fall through
			case 0xB1: /* 1011.0001 */
				set_voltage = 1;
				direction = buffer & 0x1;
				return true;

			case 0xD0: /* 1101.0000 */ toggle_led   = true;     return true;
			case 0xE0: /* 1110.0000 */ increase_pwm = true;     return true;
			case 0xF0: /* 1111.0000 */ decrease_pwm = true;     return true;

			case 0xF1: /* 1111.0001 */ release_mode = true; return true;
			default:
				//Uart0::write(buffer);
				break;
		}

		/* no command sent */
		if (set_voltage == 2) {
			target_pwm = buffer;
			set_voltage = 3;
			return true;
		}

		return false;
	}

	bool search_for_id() { return true; }//buffer == motor_id; }

	void process_command() {

		if (data_requested) {
			data_requested = false;
			position = ux.get_position();
			Uart0::write(0x80); /* 1000.0000 */
			Uart0::write((position >> 8) & 0xff); //TODO transmit, leaving MSB 0
			Uart0::write( position       & 0xff);
		}

		if (toggle_enable) {
			toggle_enable = false;
			ux.toggle_enable();
		}

		if (set_voltage == 3) {
			set_voltage = 0;
			ux.set_pwm(target_pwm);
			ux.set_dir(direction);
		}

		if (release_mode) {
			release_mode = false;
			ux.toggle_full_release();
		}

		if (increase_pwm) {
			increase_pwm = false;
			ux.inc_pwm();
		}

		if (decrease_pwm) {
			decrease_pwm = false;
			ux.dec_pwm();
		}

		if (toggle_led) { //TODO: apply pwm to LED
			toggle_led = false;
			if (led_state) {
				Board::led_D5::reset();
				led_state = false;
			}
			else {
				Board::led_D5::set();
				led_state = true;
			}
		}

	}

	void step_irq() {
		if (Uart0::read(buffer))
			search_for_command();
	}

	void step()
	{
		process_command();
	}
};

} // namespace supreme
