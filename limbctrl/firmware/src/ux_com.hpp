#ifndef SUPREME_LIMBCONTROLLER_UX_COM
#define SUPREME_LIMBCONTROLLER_UX_COM

#include <xpcc/architecture/platform.hpp>
#include <src/common.hpp>
#include <src/timer.hpp>
//#include <communication/sendbuffer.hpp>

using namespace Board;

namespace supreme {

	enum connection_status_t {
		not_connected   = 0,
		request_pending,
		responded, // waiting for timeslot to close
		is_connected,
		is_faulted,
		unknown,
	};

template <typename Interface, unsigned N>
class sendbuffer {
	static const unsigned NumSyncBytes = 2;
	static const uint8_t chk_init = 0xFE; /* (0xff + 0xff) % 256 */
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

	void add_byte(uint8_t byte) {
		assert(ptr < (N-1), 1);
		buffer[ptr++] = byte;
		checksum += byte;
	}

	void add_word(uint16_t word) {
		add_byte((word  >> 8) & 0xff);
		add_byte( word        & 0xff);
	}

	void discard(void) { ptr = NumSyncBytes; }

	void transmit() {
		if (ptr == NumSyncBytes) return;
		add_checksum();
		Interface::send_mode();
		Interface::uart::write(buffer, ptr);
		Interface::uart::flushWriteBuffer();
		Interface::recv_mode();
		/* prepare next */
		ptr = NumSyncBytes;
	}

	uint16_t size(void) const { return ptr; }

private:
	void add_checksum() {
		assert(ptr < N, 8);
		buffer[ptr++] = ~checksum + 1; /* two's complement checksum */
		checksum = chk_init;
	}
};

template <typename Interface_t, typename Timer_t>
class ux_communication_ctrl {
public:
	enum command_id_t {
		unrecognized_command,
		data_requested_response,
		ping_response,
	};

	enum recv_state_t {
		syncing   = 0,
		awaiting  = 1,
		get_id    = 2,
		reading   = 3,
		verifying = 5,
		pending   = 6,
		finished  = 7,
		error     = 8,
	};



private:

	const uint8_t                syncbyte = 0xff;

	uint8_t                      recv_buffer = 0;
	uint8_t                      recv_checksum = 0;

	sendbuffer<Interface_t, 16>  msg;
	uint8_t                      motor_id;

	/* motor related */
	bool                         target_dir = false;
	uint8_t                      target_pwm = 0;


	command_id_t                 cmd_id    = unrecognized_command;
	recv_state_t                 cmd_state = syncing;
	unsigned int                 cmd_bytes_received = 0;

	bool                         sync_state = false;

	uint16_t                     errors = 0;

	connection_status_t          connection_status = connection_status_t::unknown;

public:

	ux_communication_ctrl(uint8_t motor_id) : msg(), motor_id(motor_id) {}

	bool step(bool ux_request_timed_out)
	{
		if (ux_request_timed_out) {
			switch(connection_status)
			{
			case request_pending: connection_status = not_connected; break;
			case responded:       connection_status = is_connected;  break;
			default:
				break;
			}
			return true;
		}

		// otherwise...

		if (connection_status != connection_status_t::request_pending
		and connection_status != connection_status_t::responded ) {
			send_state_request();//send_ping();
			Timer_t:: template setPeriod<Board::systemClock>(500);
			reset_and_start_timer<Timer_t>();
			connection_status = connection_status_t::request_pending;
		} else
		{
			while(receive_response());
		}
		//led_red::toggle();
		return false;
	}

	connection_status_t get_status(void) const { return connection_status; }

	void request_data() { /*TODO implement */ }


private:


	void send_ping(void) {
		msg.add_byte(0xE0);
		msg.add_byte(motor_id);
		msg.transmit();
	}

	void send_state_request(void) {
		msg.add_byte(0xC0);
		msg.add_byte(motor_id);
		msg.transmit();
	}

	bool byte_received(void) {
		bool result = Interface_t::uart::read(recv_buffer);
		if (result)
			recv_checksum += recv_buffer;
		return result;
	}

	recv_state_t waiting_for_id()
	{
		if (recv_buffer > 127) return error;
		switch(cmd_id)
		{
			/* responses */
			case ping_response:           return verifying;
			case data_requested_response: return reading;
			default: /* unknown command */ break;
		}
		assert(false, 3);
		return finished;
	}

	recv_state_t waiting_for_data()
	{
		switch(cmd_id)
		{
			case data_requested_response:
				//TODO
				return verifying;

			default: /* unrecognized command */ break;
		}
		//assert(false, 4);
		return finished;
	}

	recv_state_t verify_checksum() { return (recv_checksum == 0) ? pending : error; }

	recv_state_t search_for_command()
	{
		switch(recv_buffer)
		{
			case 0xE1: /* 1110.0001 */ cmd_id = ping_response;           break;
			case 0x80: /* 1000.0000 */ cmd_id = data_requested_response; break;
			default: /* unrecognized command */
				return error;
		} /* switch recv_buffer */
		return get_id;
	}

	recv_state_t get_sync_bytes()
	{
		if (recv_buffer != syncbyte) {
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

	recv_state_t process_response()
	{
		//TODO
		connection_status = connection_status_t::responded;
		return finished;
	}

	/* return code true: continue processing,
	              false: wait for next byte */
	bool receive_response()
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

			case verifying:
				if (not byte_received()) return false;
				cmd_state = verify_checksum();
				break;

			case pending:
				cmd_state = process_response();
				break;

			case finished: /* cleanup, prepare for next message */
				cmd_id = unrecognized_command;
				cmd_state = syncing;
				recv_checksum = 0;
				assert(sync_state == false, 55);
				/* anything else todo? */
				break;

			case error:
				if (errors < 0xffff) ++errors;
				cmd_state = finished;
				break;

			default: /* unknown command state */
				assert(false, 17);
				break;

		} /* switch cmd_state */
		return true; // continue
	}

};

} /* namespace supreme */

#endif /* SUPREME_LIMBCONTROLLER_UX_COM */
