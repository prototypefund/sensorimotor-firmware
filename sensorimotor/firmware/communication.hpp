#include <xpcc/architecture/platform.hpp>

namespace supreme {

	inline void helloworld() {
		/* send hello */
		Uart0::write('u');
		Uart0::write('x');
		Uart0::write('0');
	}


class communication_ctrl {

	enum command_t {
		no_command     = 0,
		data_requested = 1,
		toggle_enable  = 2,
		set_voltage    = 3,
		toggle_led     = 4,
		increase_pwm   = 5,
		decrease_pwm   = 6,
		release_mode   = 7,
	};


	supreme::sensorimotor_core& ux;
	uint8_t buffer;
	uint16_t position;
	bool     direction;

	const uint8_t motor_id = 17;

	command_t command;

	bool led_state = false;

public:

	communication_ctrl(supreme::sensorimotor_core& ux)
	: ux(ux)
	, position()
	, direction()
	, command(no_command)
	{}


	bool search_for_command() {

		switch(buffer) {
			case 0xC0: /* 1100.0000 */ command = data_requested;   return true;
			case 0xA0: /* 1010.0000 */ command = toggle_enable;    return true;

			case 0xB0: /* 1011.0000 */ //fall through
			case 0xB1: /* 1011.0001 */
				command = set_voltage;
				direction = buffer & 0x1;
				return true;

			case 0xD0: /* 1101.0000 */ command = toggle_led;       return true;
			case 0xE0: /* 1110.0000 */ command = increase_pwm;     return true;
			case 0xF0: /* 1111.0000 */ command = decrease_pwm;     return true;

			case 0xF1: /* 1111.0001 */ command = release_mode;     return true;
			default:
				//Uart0::write(buffer);
				break;
		}
		return false;
	}

	bool search_for_id() { return true; }//buffer == motor_id; }

	void process_command() {

		switch(command) {
			case data_requested:
				position = ux.get_position();
				Uart0::write(0x80);
				Uart0::write((position >> 8) & 0xff); //TODO transmit, leaving MSB 0
				Uart0::write( position       & 0xff);
				break;

			case toggle_enable:
				ux.toggle_enable();
				break;

			case set_voltage:
				if (Uart0::read(buffer)) {
					ux.set_pwm(buffer);
					ux.set_dir(direction);
				}
				break;

			case release_mode:
				ux.toggle_full_release();
				break;

			case increase_pwm:
				ux.inc_pwm();
				break;

			case decrease_pwm:
				ux.dec_pwm();
				break;

			case toggle_led: //TODO: apply pwm to LED
				if (led_state) {
					Board::led_D5::reset();
					led_state = false;
				}
				else {
					Board::led_D5::set();
					led_state = true;
				}
				break;

			default:
				break;
		}
		command = no_command;
	}

	void step()
	{
		/**TODO: split up receiving cmd/data and executing
		 * receive and empty the usart recv buffer 1kHz loop
		 * process and execute command in 100 Hz loop, or as
		 * fast as possible */

		if (Uart0::read(buffer)) {
			search_for_command();
		}
		process_command();
	}
};

} // namespace supreme
