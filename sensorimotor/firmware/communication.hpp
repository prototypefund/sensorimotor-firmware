#include <xpcc/architecture/platform.hpp>
#include <avr/eeprom.h>
#include <assert.hpp>
/*
TODO: create new scheme for command processing:

	0) get sync bytes
	1) detect command
	2) look up expected number of bytes
	3) read all bytes (including checksum)
	4) verify checksum
	5) process command (or discard)
		+ dicard if
			- ID does not match
			- checksum is incorrect
			- timeout in byte stream

	consider having a class for each command, derived from a (virtual) base class

	TODO: clear recv buffer after timeout
*/
namespace supreme {


template <unsigned N, unsigned NumSyncBytes = 2>
class sendbuffer {
	static const uint8_t chk_init = 0xFE; /* (0xff + 0xff) % 256*/
	uint16_t  ptr = NumSyncBytes;
	uint8_t   buffer[N];
	uint8_t   checksum = chk_init;
public:

	sendbuffer()
	{
		static_assert(N > NumSyncBytes, "Invalid buffer size.");
		for (uint8_t i = 0; i < NumSyncBytes; ++i)
			buffer[i] = 0xFF; // init sync bytes once
	}

	void add(uint8_t byte) {
		assert(ptr < (N-1), 1);
		buffer[ptr++] = byte;
		checksum += byte;
	}

	void flush() {
		if (ptr == NumSyncBytes) return;
		add_checksum();
		send_mode();
		Uart0::write(buffer, ptr);
		Uart0::flushWriteBuffer();
		receive_mode();

		/* prepare next */
		ptr = NumSyncBytes;
	}

private:
	void add_checksum() {
		assert(ptr < N, 8);
		buffer[ptr++] = ~checksum + 1; /* two's complement checksum */
		checksum = chk_init;
	}

	void send_mode() {
		xpcc::delayNanoseconds(50); // wait for signal propagation
		rs485::read_disable::set();
		rs485::drive_enable::set();
		xpcc::delayMicroseconds(1); // wait at least one bit after enabling the driver
	}

	void receive_mode() {
		xpcc::delayMicroseconds(1); // wait at least one bit before disabling the driver
		rs485::read_disable::reset();
		rs485::drive_enable::reset();
		xpcc::delayNanoseconds(70); // wait for signal propagation
	}
};

class communication_ctrl {

	enum command_id_t { //TODO: this should be classes
		no_command,
		data_requested,
		data_requested_response,
		toggle_enable,
		set_voltage,
		toggle_led,
		release_mode,
		ping,
		ping_response,
		set_id,
		set_id_response,
	};

	enum command_state_t {
		syncing,
		awaiting,
		get_id,
		reading,
		eating,
		verifying,
		pending,
		finished,
		error
	};

	supreme::sensorimotor_core&  ux;
	uint8_t                      buffer = 0; //TODO rename to recv_buffer
	uint8_t                      recv_checksum = 0;
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

	uint8_t                      num_bytes_eaten = 0;
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

	bool byte_received(void) {
		bool result = Uart0::read(buffer);
		if (result)
			recv_checksum += buffer;
		return result;
	}

	command_state_t waiting_for_id()
	{
		if (buffer > 127) return error;
		switch(cmd_id)
		{
			case data_requested:
			case toggle_enable:
			case release_mode:
			case toggle_led:
			case ping:
				return (motor_id == buffer) ? verifying : eating;

			case set_voltage:
			case set_id:
				return (motor_id == buffer) ? reading : eating;

			/* responses */
			case ping_response:           return eating;
			case set_id_response:         return eating;
			case data_requested_response: return eating;

			default: /* unknown command */ break;
		}
		assert(false, 3);
		return finished;
	}

	command_state_t process_command()
	{
		switch(cmd_id)
		{
			case data_requested:
				position = ux.get_position();
				send.add(0x80); /* 1000.0000 */
				send.add(motor_id);
				send.add((position >> 8) & 0xff);
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
				send.add(0x71); /* 0111.0001 */
				send.add(motor_id);
				break;

			default: /* unknown command */
				assert(false, 2);
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
				return verifying;

			case set_id:
				target_id = buffer;
				return verifying;

			default: /* unknown command */ break;
		}
		assert(false, 4);
		return finished;
	}

	command_state_t eating_others_data()
	{
		++num_bytes_eaten;
		switch(cmd_id)
		{
			case data_requested:
			case toggle_enable:
			case release_mode:
			case toggle_led:
			case ping:
				return finished;

			case set_voltage:
			case set_id:
			case ping_response:
			case set_id_response:
				return (num_bytes_eaten == 1) ? finished : eating;

			case data_requested_response:
				return (num_bytes_eaten == 2+1) ? finished : eating;

			default: /* unknown command */ break;
		}
		assert(false, 5);
		return finished;
	}

	command_state_t verify_checksum()
	{
		return (recv_checksum == 0) ? pending : finished;
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
			case 0x70: /* 0111.0000 */ cmd_id = set_id;          break;

			/* read but ignore sensorimotor responses */
			case 0xE1: /* 1110.0001 */ cmd_id = ping_response;   break;
			case 0x71: /* 0111.0001 */ cmd_id = set_id_response; break;
			case 0x80: /* 1000.0000 */ cmd_id = data_requested_response; break;

			default: /* unknown command */
				return error;

		} /* switch buffer */

		return get_id;
	}

	/* return code true means continue processing, false: wait for next byte */
	bool receive_command()
	{
		switch(cmd_state)
		{
			case syncing:
				if (not byte_received()) return false;
				cmd_state = get_sync_bytes();
				break;

			case awaiting:
				if (not byte_received()) return false;
				cmd_state = search_for_command();
				break;

			case get_id:
				if (not byte_received()) return false;
				cmd_state = waiting_for_id();
				break;

			case reading:
				if (not byte_received()) return false;
				cmd_state = waiting_for_data();
				break;

			case eating:
				if (not byte_received()) return false;
				cmd_state = eating_others_data();
				break;

			case verifying:
				if (not byte_received()) return false;
				cmd_state = verify_checksum();
				break;

			case pending:
				cmd_state = process_command();
				break;

			case finished:
				send.flush();
				cmd_id = no_command;
				cmd_state = syncing;
				num_bytes_eaten = 0;
				recv_checksum = 0;
				/* anything else todo? */
				break;

			case error: //TODO goto finished?
				led::yellow::set();
				cmd_id = no_command;
				cmd_state = syncing;
				num_bytes_eaten = 0;
				recv_checksum = 0;
				break;

			default: /* unknown command state */
				assert(false, 17);
				break;

		} /* switch cmd_state */
		return true; // continue
	}

	void step() {
		while(receive_command());
	}
};

} // namespace supreme
