/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCONTROLLER_MOTORCORD
#define SUPREME_LIMBCONTROLLER_MOTORCORD

#include <src/ux_com.hpp>

namespace supreme {

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


	MotorCord(target_voltage_t& voltages)
	: voltages(voltages)
	{
		// TODO ping motors...and check if motors are connected correctly.
	}

	void prepare(void) {
		for (auto& m: motors) {
			if (m.get_id() < 12)
				m.set_target_voltage(voltages[m.get_id()]);
			else
				m.set_target_voltage(0);
		}
		idx = 0;
		state = ready;
		voltages.fill(0); // clear target voltages;
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

	target_voltage_t& voltages;

}; /* class MotorCord */

} /* namespace supreme */

#endif /* SUPREME_LIMBCONTROLLER_MOTORCORD */
