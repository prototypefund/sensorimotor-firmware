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
		ping,
	};

	enum command_state_t {
		awaiting,
		get_id,
		reading,
		eating,
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


	command_state_t waiting_for_id()
	{
		switch(cmd_id)
		{
			case data_requested:
			case toggle_enable:
			case release_mode:
			case toggle_led:
			case ping:
				return (motor_id == buffer) ? pending : finished;

			case set_voltage:
				return (motor_id == buffer) ? reading : eating;

			default: /* unknown command */
				return finished;
		}
	}

	command_state_t process_command()
	{
		switch(cmd_id)
		{
			case data_requested:
				position = ux.get_position();
				Uart0::write(0x80); /* 1000.0000 */
				Uart0::write((position >> 8) & 0xff); //TODO transmit, leaving MSB 0
				Uart0::write( position       & 0xff);
				break;

			case toggle_enable:
				ux.toggle_enable();
				break;

			case set_voltage:
				ux.set_pwm(target_pwm);
				ux.set_dir(direction);
				break;

			case release_mode:
				ux.toggle_full_release();
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

			case ping:
				Uart0::write(0xE1); /* 1110.0001 */
				Uart0::write(motor_id);
				break;

			default: /* unknown command */
				break;

		} /* switch cmd_id */

		return finished;
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
			case 0xC0: /* 1100.0000 */ cmd_id = data_requested;  break;
			case 0xA0: /* 1010.0000 */ cmd_id = toggle_enable;   break;
			case 0xD0: /* 1101.0000 */ cmd_id = toggle_led;      break;
			case 0xF1: /* 1111.0001 */ cmd_id = release_mode;    break;

			/* multi-byte commands */
			case 0xB0: /* 1011.0000 */ //fall through
			case 0xB1: /* 1011.0001 */ cmd_id = set_voltage;
			                           direction = buffer & 0x1; break;
			case 0xE0: /* 1110.0000 */ cmd_id = ping;            break;

			default: /* unknown command */
				return finished;

		} /* switch buffer */

		return get_id;
	}


	void receive_command()
	{
		switch(cmd_state)
		{
			case awaiting:
				if (Uart0::read(buffer))
					cmd_state = search_for_command();
				return;

			case get_id:
				if (Uart0::read(buffer))
					cmd_state = waiting_for_id();
				return;

			case reading:
				if (Uart0::read(buffer))
					cmd_state = waiting_for_data();
				return;

			case eating:
				if (Uart0::read(buffer))
					cmd_state = finished; //eating_others_data();
				return;

			case pending:
				cmd_state = process_command();
				return;

			case finished:
				cmd_id = no_command;
				cmd_state = awaiting;
				/* anything else todo? */
				return;

			default: /* unknown command state */
				return;

		} /* switch cmd_state */
		//TODO: return true if finished, false if pending, reading...
	}


	void step_irq() {
		receive_command();
	}


	void step() {
		/* currently nothing to do at 100 Hz */
	}
};

} // namespace supreme
