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
		responded, // waiting for timeslot to close
		is_connected,
		is_faulted,
		unknown,
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
		read_id   = 2,
		reading   = 3,
		verifying = 5,
		pending   = 6,
		finished  = 7,
		error     = 8,
	};

	struct StatusData_t {
		uint16_t position;
		//TODO velocity
		uint16_t current;
		uint16_t voltage_back_emf;
		uint16_t voltage_supply;
		uint16_t temperature;
	};

private:

	const uint8_t                syncbyte = 0xff;

	typedef recvbuffer<Interface_t, 32> RecvBuffer_t;
	typedef sendbuffer<Interface_t, 16> SendBuffer_t;

	RecvBuffer_t                 recv_msg;
	SendBuffer_t                 send_msg;
	uint8_t                      motor_id;
	StatusData_t                 status_data;

	/* motor related */
	pwm_t                        target_pwm = {0, false};

	command_id_t                 cmd_id    = unrecognized_command;
	recv_state_t                 cmd_state = syncing;
	unsigned int                 cmd_bytes_received = 0;

	bool                         sync_state = false;

	uint16_t                     errors = 0;
	connection_status_t          connection_status = connection_status_t::unknown;


public:

	ux_communication_ctrl(uint8_t motor_id) : send_msg(), motor_id(motor_id), status_data() {
		assert(motor_id < 127, 6);
	}

	void set_target_voltage(scdata_t target_voltage) {
		target_pwm = sc_to_pwm(target_voltage);
	}

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
			//send_ping();
			//send_state_request();
			send_motor_request();
			Timer_t:: template setPeriod<Board::systemClock>(500);
			reset_and_start_timer<Timer_t>();
			connection_status = connection_status_t::request_pending;
		} else
		{
			while(receive_response());
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

	recv_state_t waiting_for_id()
	{
		if (recv_msg.data > 127) return error;
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
				return (recv_msg.bytes_received() < 14 /*excl. checksum*/) ? reading : verifying;

			default: /* unrecognized command */ break;
		}
		assert(false, 4);
		return finished;
	}

	recv_state_t verify_checksum() { return (recv_msg.checksum == 0) ? pending : error; }

	recv_state_t search_for_command()
	{
		switch(recv_msg.data)
		{
			case 0xE1: /* 1110.0001 */ cmd_id = ping_response;           break;
			case 0x80: /* 1000.0000 */ cmd_id = data_requested_response; break;
			default: /* unrecognized command */
				return error;
		} /* switch recv.data */
		return read_id;
	}

	recv_state_t get_sync_bytes()
	{
		if (recv_msg.data != syncbyte) {
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
		status_data.position         = recv_msg.get_word( 4); //TODO velocity
		status_data.current          = recv_msg.get_word( 6);
		status_data.voltage_back_emf = recv_msg.get_word( 8);
		status_data.voltage_supply   = recv_msg.get_word(10);
		status_data.temperature      = recv_msg.get_word(12);
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

/*
formula: i/2 * 6 + i%2 + 2*j

id  |  motors
----+------------
 0  |  0,  2,  4
 1  |  1,  3,  5
 2  |  6,  8, 10
 3  |  7,  9, 11
(4) | 12, 14, 16
(5) | 13, 15, 17
*/

constexpr
uint8_t get_motor_id_from_board_id(uint8_t board_id, uint8_t motor_index) {
	return board_id/2 * 6 + board_id%2 + 2*motor_index;
}

template <typename InterfaceType, typename TimerType, unsigned BoardID, unsigned NumMotors>
class MotorCord {

	typedef supreme::ux_communication_ctrl<InterfaceType, TimerType> sensorimotor_t;
	typedef std::array<sensorimotor_t,NumMotors> motorarray_t;

public:

	typedef std::array<scdata_t, 12> target_voltage_t;

	enum State_t {
		ready = 0,
		pending,
		waiting_for_next,
		done,
	};


	MotorCord(target_voltage_t const& voltages)
	: voltages(voltages)
	{
		// TODO ping motors...and check if motors are connected correctly.
	}

	void prepare(void) {
		for (auto& m: motors)
			m.set_target_voltage(voltages[m.get_id()]);
		idx = 0;
		state = ready;
	}

	/* after preparing motor commands,
	   transmit() can be called multiple times,
	   it returns true times the number of motors */
	State_t transmit(bool is_timeout_out)
	{
		assert(idx < NumMotors, 7);

		if (motors[idx].step(is_timeout_out)) {
			++idx;
			state = (idx < NumMotors) ? waiting_for_next : done;
		} else
			state = pending;

		return state;
	}

	motorarray_t const& get_motors(void) const { return motors; }

private:

	unsigned idx = 0;
	State_t state = done; // prepare_motor_commands() must be called first


	motorarray_t motors = { get_motor_id_from_board_id(BoardID, 0)
	                      , get_motor_id_from_board_id(BoardID, 1)
	                      , get_motor_id_from_board_id(BoardID, 2) };

	target_voltage_t const& voltages;
};

} /* namespace supreme */

#endif /* SUPREME_LIMBCONTROLLER_UX_COM */
