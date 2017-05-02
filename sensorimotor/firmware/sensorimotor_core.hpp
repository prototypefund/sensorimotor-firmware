#include <xpcc/architecture/platform.hpp>

#include <constants.hpp>
#include <read_sensor.hpp>
#include <motor_ifx9201sg.hpp>

//using namespace xpcc::atmega;

namespace supreme {

class sensorimotor_core {

	bool dir_left;
	bool enabled;

	uint8_t voltage_pwm;

	read_sensor      sensor;
	motor_ifx9201sg  motor;

	bool release_mode;
	uint16_t last_position;

public:

	sensorimotor_core()
	: dir_left(true)
	, enabled(false)
	, voltage_pwm(0)
	, sensor()
	, motor()
	, release_mode()
	, last_position(sensor.get_value())
	{
		motor.disable();
		motor.set_pwm(voltage_pwm);
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

		/** release_mode */
		if (release_mode) {
			if (last_position < value) {
				set_dir(false);
				set_pwm(20);
			} else if (last_position > value) {
				set_dir(true);
				set_pwm(20);
			} else
				set_pwm(0);
		}
		last_position = value;
	}

	//void inc_pwm() { if (voltage_pwm < 128) motor.set_pwm(++voltage_pwm); }
	//void dec_pwm() { if (voltage_pwm > 0  ) motor.set_pwm(--voltage_pwm); }

	void set_pwm(uint8_t pwm) { if (pwm <= 128  ) motor.set_pwm(pwm); }

	void set_dir(bool dir) {
		dir_left = dir;
		motor.set_direction(dir);
	}

	void toggle_enable() {
		enabled = !enabled;
		if (enabled) motor.enable();
		else motor.disable();
	}

	void enable()  { motor.enable();  }
	void disable() { motor.disable(); }

	void toggle_full_release() { release_mode = not release_mode; }

	uint16_t get_position() { return sensor.get_value(); }
};

} /* namespace supreme */
