/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCONTROLLER_UX_COM
#define SUPREME_LIMBCONTROLLER_UX_COM

#include <array>

#include <xpcc/architecture/platform.hpp>
#include <src/common.hpp>
#include <src/timer.hpp>
#include <src/math.hpp>
#include <src/transceivebuffer.hpp>

using namespace Board;

namespace supreme {

	enum connection_status_t {
		not_connected   = 0,
		request_pending,
		responded,
		waiting,
		is_connected,
	};

template <typename Interface_t, typename Timer_t>
class ux_communication_ctrl {
public:

	static const unsigned motor_timeout_us = 500;
	static const unsigned wait_before_next_us = 100;

	enum request_id_t {
		ping,
		data_requested,
		set_voltage,
		set_pwm_limit, /* no response */
		ext_sensor_request,
	};

	enum response_id_t {
		unrecognized_command,
		data_requested_response,
		ping_response,
		ext_sensor_request_resp,
	};

	enum recv_state_t {
		syncing   = 0,
		awaiting  = 1,
		read_id   = 2,
		reading   = 3,
		verifying = 5,
		pending   = 6,
		finished  = 7,
		error     = 8,
	};

	struct StatusData_t {
		/*---------------------------------------------------------------+
		 | sensorimotor status bits                                      |
		 +-------------------------------+-------------------------------+
		 | 0: motor is_connected         | 8: external sensor connected  |
		 | 1: reserved                   | 9: reserved                   |
		 | 2: reserved                   | A: reserved                   |
		 | 3: reserved                   | B: reserved                   |
		 | 4: reserved                   | C: reserved                   |
		 | 5: reserved                   | D: reserved                   |
		 | 6: reserved                   | E: reserved                   |
		 | 7: reserved                   | F: reserved                   |
		 +-------------------------------+-------------------------------*/
		//TODO: uint16_t status         = 0;
		uint16_t position       = 0;
		uint16_t velocity       = 0;
		uint16_t current        = 0;
		uint16_t voltage_supply = 0;
		uint16_t temperature    = 0;
		uint16_t ext_sensor[3]  = {0,0,0};
		//uint16_t voltage_back_emf; <-- currently not in use
	};

private:

	static const uint8_t syncbyte = 0xff;

	typedef recvbuffer<Interface_t, 32          > RecvBuffer_t;
	typedef sendbuffer<Interface_t, 16, syncbyte> SendBuffer_t;

	RecvBuffer_t                 recv_msg;
	SendBuffer_t                 send_msg;
	uint8_t                      motor_id;
	StatusData_t                 status_data;

	/* motor related */
	pwm_t                        target_pwm = {0, false};
	const uint8_t                limit_pwm = 128; //TODO include in transparent data?

	response_id_t                cmd_id    = unrecognized_command;
	recv_state_t                 cmd_state = syncing;
	unsigned int                 cmd_bytes_received = 0;

	bool                         sync_state = false;
	bool                         readout_ext_sensor = false;

	uint16_t                     errors = 0;
	connection_status_t          connection_status = connection_status_t::not_connected;

public:

	ux_communication_ctrl(uint8_t motor_id) : send_msg(), motor_id(motor_id), status_data() {
		assert(motor_id < 127, 6);
	}

	void enable_ext_sensor_reading(bool enable = true) { readout_ext_sensor = enable; }

	bool read_ext_sensor(volatile bool* is_timed_out) {
		if (not readout_ext_sensor) return true; // done
		return step(is_timed_out, ext_sensor_request);
	}

	bool ping_and_setup(volatile bool* is_timed_out) {
		*is_timed_out = false;
		assert(*is_timed_out == false, 0xAA);
		assert(connection_status == not_connected, 0xAB);
		while(!step(is_timed_out, ping));
		if (connection_status != is_connected)
			return false; // abort further setup

		send_voltage_limit();
		*is_timed_out = false;
		start_timer(wait_before_next_us);
		while(not *is_timed_out);
		return true;
	}

	void set_target_voltage(scdata_t target_voltage) {
		target_pwm = sc_to_pwm(target_voltage);
	}

	bool step(volatile bool* is_timed_out, request_id_t req_id = request_id_t::set_voltage)
	{
		if (*is_timed_out)
		{
			switch(connection_status)
			{
			case request_pending: connection_status = not_connected; break;
			case waiting:         connection_status =  is_connected; break;
			default:
				assert(false, 78);
				break;
			}
			return true;
		}

		// otherwise...timer still active
		switch(connection_status)
		{
		case waiting: /* waiting */ return false;

		case responded:
			start_timer(wait_before_next_us);
			connection_status = waiting;
			return false;

		case request_pending:
			while(receive_response());
			break;

		case not_connected: /* fall through */
		case is_connected:
			select_command_and_start_timer(req_id);
			connection_status = request_pending;
			break;

		default: /* unknown state */
			assert(false, 79);
			break;
		}
		return false;
	}

	connection_status_t get_connection_status(void) const { return connection_status; }
	StatusData_t const& get_status_data(void) const { return status_data; }

	uint8_t get_id(void) const { return motor_id; }

private:


	void send_ping(void) {
		send_msg.add_byte(0xE0);
		send_msg.add_byte(motor_id);
		send_msg.transmit();
	}

	void send_state_request(void) {
		send_msg.add_byte(0xC0);
		send_msg.add_byte(motor_id);
		send_msg.transmit();
	}

	void send_motor_request(void) {
		const uint8_t cmd = target_pwm.dir ? 0xB1 : 0xB0;
		send_msg.add_byte(cmd);
		send_msg.add_byte(motor_id);
		send_msg.add_byte(target_pwm.dc);
		send_msg.transmit();
	}

	void send_voltage_limit(void) {
		send_msg.add_byte(0xA0);
		send_msg.add_byte(motor_id);
		send_msg.add_byte(limit_pwm);
		send_msg.transmit();
	}

	void send_ext_sensor_req(uint8_t sensor_id = 1) {
		send_msg.add_byte(0x40);
		send_msg.add_byte(motor_id);
		send_msg.add_byte(sensor_id);
		send_msg.transmit();
	}

	void select_command_and_start_timer(request_id_t req_id) {
		switch(req_id) { /* select command */
		case ping               : send_ping();           break;
		case data_requested     : send_state_request();  break;
		case set_voltage        : send_motor_request();  break;
		case ext_sensor_request : send_ext_sensor_req(); break;
		case set_pwm_limit      : /* not allowed to call this way */
		default                 : assert(false, 76);     break;
		}

		start_timer(motor_timeout_us);
	}

	void start_timer(unsigned time_us) {
		Timer_t:: template setPeriod<Board::systemClock>(time_us);
		reset_and_start_timer<Timer_t>();
	}

	recv_state_t waiting_for_id()
	{
		if (recv_msg.get_data() > 127) return error;
		switch(cmd_id)
		{
			/* responses */
			case ping_response:           return verifying;
			case data_requested_response: return reading;
			case ext_sensor_request_resp: return reading;
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
				return (recv_msg.bytes_received() < 14 /*excl. checksum*/) ? reading : verifying;
			case ext_sensor_request_resp:
				return (recv_msg.bytes_received() < 10 /*excl. checksum*/) ? reading : verifying;
			default: /* unrecognized command */
				break;
		}
		assert(false, 4);
		return finished;
	}

	recv_state_t verify_checksum() { return recv_msg.verify() ? pending : error; }

	recv_state_t search_for_command()
	{
		switch(recv_msg.get_data())
		{
			case 0xE1: /* 1110.0001 */ cmd_id = ping_response;           break;
			case 0x80: /* 1000.0000 */ cmd_id = data_requested_response; break;
			case 0x41: /* 0100.0001 */ cmd_id = ext_sensor_request_resp; break;
			default: /* unrecognized command */
				return error;
		} /* switch recv.data */
		return read_id;
	}

	recv_state_t get_sync_bytes()
	{
		if (recv_msg.get_data() != syncbyte) {
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
		switch(cmd_id)
		{
			case ping_response:
				connection_status = connection_status_t::responded;
				break;

			case data_requested_response:
				status_data.position         = recv_msg.get_word( 4);
				status_data.current          = recv_msg.get_word( 6);
				status_data.velocity         = recv_msg.get_word( 8);
				status_data.voltage_supply   = recv_msg.get_word(10);
				status_data.temperature      = recv_msg.get_word(12);
				//TODO add voltage_backemf and target voltage readback
				connection_status = connection_status_t::responded;
				break;

			case ext_sensor_request_resp:
				status_data.ext_sensor[0] = recv_msg.get_word(4);
				status_data.ext_sensor[1] = recv_msg.get_word(6);
				status_data.ext_sensor[2] = recv_msg.get_word(8);
				connection_status = connection_status_t::responded;
				break;

			default: /* unknown command */
				assert(false, 27);
				break;

		} /* switch cmd_id */
		return finished;
	}

	/* return code true: continue processing,
	              false: wait for next byte */
	bool receive_response()
	{
		switch(cmd_state)
		{
			case syncing:
				if (not recv_msg.read_byte()) return false;
				cmd_state = get_sync_bytes();
				break;

			case awaiting:
				if (not recv_msg.read_byte()) return false;
				cmd_state = search_for_command();
				break;

			case read_id:
				if (not recv_msg.read_byte()) return false;
				cmd_state = waiting_for_id();
				break;

			case reading:
				if (not recv_msg.read_byte()) return false;
				cmd_state = waiting_for_data();
				break;

			case verifying:
				if (not recv_msg.read_byte()) return false;
				cmd_state = verify_checksum();
				break;

			case pending:
				cmd_state = process_response();
				break;

			case finished: /* cleanup, prepare for next message */
				cmd_id = unrecognized_command;
				cmd_state = syncing;
				recv_msg.reset();
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
