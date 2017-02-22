#include <xpcc/architecture/platform.hpp>

#include <constants.hpp>
#include <read_sensor.hpp>
#include <motor_ifx9201sg.hpp>

//using namespace xpcc::atmega;

namespace supreme {

class sensorimotor_core {

	bool dir_left;

	read_sensor      sensor;
	motor_ifx9201sg  motor;

public:

	sensorimotor_core()
	: dir_left(true)
	, sensor()
	, motor()
	{
		motor.enable();
		motor.set_pwm(96);
	}

	void step() {

		sensor.step();
		const uint16_t value = sensor.get_value();

		/* simple test controller, toggles between max bounds */
		if (dir_left && value < lower_bound) {
			dir_left = false;
			motor.toggle_direction();
		} else if (!dir_left && value > upper_bound){
			dir_left = true;
			motor.toggle_direction();
		}

		/* out put sensor value */
		//serialStream << value << "\n\r";//xpcc::endl;
		serialStream << "\r" << value;
	}

};

} /* namespace supreme */
