#include <xpcc/architecture/platform.hpp>

namespace supreme {

	inline void helloworld() {
		/* send hello */
		Uart0::write('u');
		Uart0::write('x');
		Uart0::write('0');
	}


class communication_ctrl {

	enum command_id_t {
		no_command,
		data_requested,
		toggle_enable,
		set_voltage,
		toggle_led,
		release_mode,
	};

	enum command_state_t {
		awaiting,
		reading,
		pending,
		finished
	};

	supreme::sensorimotor_core&  ux;
	uint8_t                      buffer;

	const uint8_t                motor_id = 17;

	/* motor related */
	bool                         direction  = false;
	uint8_t                      target_pwm = 0;
	uint16_t                     position   = 0;

	/* TODO struct? */
	command_id_t                 cmd_id    = no_command;
	command_state_t              cmd_state = awaiting;
	unsigned int                 cmd_bytes_received = 0;

	bool                         led_state = false;

public:

	communication_ctrl(supreme::sensorimotor_core& ux)
	: ux(ux)
	{}


	command_state_t process_command()
	{
		switch(cmd_id)
		{
			case data_requested:
				position = ux.get_position();
				Uart0::write(0x80); /* 1000.0000 */
				Uart0::write((position >> 8) & 0xff); //TODO transmit, leaving MSB 0
				Uart0::write( position       & 0xff);
				return finished;

			case toggle_enable:
				ux.toggle_enable();
				return finished;

			case set_voltage:
				ux.set_pwm(target_pwm);
				ux.set_dir(direction);
				return finished;

			case release_mode:
				ux.toggle_full_release();
				return finished;

			case toggle_led: //TODO: apply pwm to LED
				if (led_state) {
					Board::led_D5::reset();
					led_state = false;
				}
				else {
					Board::led_D5::set();
					led_state = true;
				}
				return finished;

			default: /* unknown command */
				return finished;

		} /* switch cmd_id */
	}


	/* handle multi-byte commands */
	command_state_t waiting_for_data()
	{
		switch(cmd_id)
		{
			case set_voltage:
				target_pwm = buffer;
				return pending;

			default: /* unknown command */
				return finished;
		}
	}


	command_state_t search_for_command()
	{
		switch(buffer)
		{
			/* single byte commands */
			case 0xC0: /* 1100.0000 */ cmd_id = data_requested;  return pending;
			case 0xA0: /* 1010.0000 */ cmd_id = toggle_enable;   return pending;
			case 0xD0: /* 1101.0000 */ cmd_id = toggle_led;      return pending;
			case 0xF1: /* 1111.0001 */ cmd_id = release_mode;    return pending;

			/* multi-byte commands */
			case 0xB0: /* 1011.0000 */ //fall through
			case 0xB1: /* 1011.0001 */ cmd_id = set_voltage;
			                           direction = buffer & 0x1; return reading;

			default: /* unknown command */
				return finished;

		} /* switch buffer */
	}


	void receive_command()
	{
		switch(cmd_state)
		{
			case awaiting:
				if (Uart0::read(buffer))
					cmd_state = search_for_command();
				return;

			case reading:
				if (Uart0::read(buffer))
					cmd_state = waiting_for_data();
				return;

			case pending:
				cmd_state = process_command();
				return;

			case finished:
				cmd_id = no_command;
				cmd_state = awaiting;
				/* anything else todo? */

			default: /* unknown command state */
				return;

		} /* switch cmd_state */
	}


	void step_irq() {
		receive_command();
	}


	void step() {
		/* currently nothing to do at 100 Hz */
	}
};

} // namespace supreme
