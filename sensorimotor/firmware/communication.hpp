#include <xpcc/architecture/platform.hpp>
#include <avr/eeprom.h>
/*TODO create command to write new id to eeprom, standard ID is max. ID*/

namespace supreme {


template <unsigned N>
class sendbuffer {
	uint16_t  ptr = 0;
	uint8_t   buffer[N];
public:
	void add(uint8_t byte) {
		if (ptr < N)
			buffer[ptr++] = byte;
	}
	void flush() {
		if (ptr == 0) return;
		rs485::read_disable::set();
		rs485::drive_enable::set();
		for (uint16_t i = 0; i < ptr; ++i)
			Uart0::write(buffer[i]);
		//Uart0::write(buffer, ptr);
		ptr = 0;
		Uart0::flushWriteBuffer();
		xpcc::delayMilliseconds(1);
		rs485::drive_enable::reset();
		rs485::read_disable::reset();
	}
};

class communication_ctrl {

	enum command_id_t {
		no_command,
		data_requested,
		toggle_enable,
		set_voltage,
		toggle_led,
		release_mode,
		ping,
		set_id,
	};

	enum command_state_t {
		syncing,
		awaiting,
		get_id,
		reading,
		eating,
		pending,
		finished
	};

	supreme::sensorimotor_core&  ux;
	uint8_t                      buffer = 0; //TODO rename to recv_buffer
	sendbuffer<16>               send;

	uint8_t                      motor_id = 127; // set to default
	uint8_t                      target_id = 127;

	/* motor related */
	bool                         direction  = false;
	uint8_t                      target_pwm = 0;
	uint16_t                     position   = 0;

	/* TODO struct? */
	command_id_t                 cmd_id    = no_command;
	command_state_t              cmd_state = syncing;
	unsigned int                 cmd_bytes_received = 0;

	bool                         led_state = false;
	bool                         sync_state = false;

public:

	communication_ctrl(supreme::sensorimotor_core& ux)
	: ux(ux)
	, send()
	{
		read_id_from_EEPROM();

		rs485::drive_enable::setOutput();
		rs485::drive_enable::reset();

		rs485::read_disable::setOutput();
		rs485::read_disable::reset();
	}

	void read_id_from_EEPROM() {
		eeprom_busy_wait();
		uint8_t read_id = eeprom_read_byte((uint8_t*)23);
		if (read_id)
			motor_id = read_id & 0x7F;
	}

	void write_id_to_EEPROM(uint8_t new_id) {
		eeprom_busy_wait();
		eeprom_write_byte((uint8_t*)23, (new_id | 0x80));
	}

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
			case set_id:
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
				send.add(0x80); /* 1000.0000 */
				send.add(motor_id);
				send.add((position >> 8) & 0xff); //TODO transmit, leaving MSB 0
				send.add( position       & 0xff);
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
					led::yellow::reset();
					led_state = false;
				}
				else {
					led::yellow::set();
					led_state = true;
				}
				break;

			case ping:
				send.add(0xE1); /* 1110.0001 */
				send.add(motor_id);
				break;

			case set_id:
				write_id_to_EEPROM(target_id);
				read_id_from_EEPROM();
				send.add(0x71); /* 1011.0001 */
				send.add(motor_id);
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

			case set_id:
				target_id = buffer;
				return pending;

			default: /* unknown command */
				return finished;
		}
	}

	command_state_t get_sync_bytes()
	{
		if (buffer != 0xFF) {
			sync_state = false;
			return finished;
		}

		if (sync_state) {
			sync_state = false;
			return awaiting;
		}

		sync_state = true;
		return syncing;
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
			case 0x70: /* 1000.0000 */ cmd_id = set_id;          break;

			default: /* unknown command */
				return finished;

		} /* switch buffer */

		return get_id;
	}


	bool receive_command()
	{
		switch(cmd_state)
		{
			case syncing:
				if (!Uart0::read(buffer)) return false;
				cmd_state = get_sync_bytes();
				break;

			case awaiting:
				if (!Uart0::read(buffer)) return false;
				cmd_state = search_for_command();
				break;

			case get_id:
				if (!Uart0::read(buffer)) return false;
				cmd_state = waiting_for_id();
				break;

			case reading:
				if (!Uart0::read(buffer)) return false;
				cmd_state = waiting_for_data();
				break;

			case eating:
				if (!Uart0::read(buffer)) return false;
				cmd_state = finished; //eating_others_data();
				break;

			case pending:
				cmd_state = process_command();
				break;

			case finished:
				send.flush();
				cmd_id = no_command;
				cmd_state = syncing;
				/* anything else todo? */
				break;

			default: /* unknown command state */
				break;

		} /* switch cmd_state */
		return true;
	}

	void step() {
		while(receive_command());
	}
};

} // namespace supreme
