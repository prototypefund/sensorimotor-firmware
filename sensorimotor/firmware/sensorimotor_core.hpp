#include <xpcc/architecture/platform.hpp>

#include <constants.hpp>
#include <read_sensor.hpp>
#include <motor_ifx9201sg.hpp>

namespace supreme {

struct Sensors {
	read_sensor<1> position;
	read_sensor<7> current;
	read_sensor<3> voltage_back_emf;
	read_sensor<6> voltage_supply;
	read_sensor<2> temperature;

	void step(void) {
		position        .step();
		current         .step();
		voltage_back_emf.step();
		voltage_supply  .step();
		temperature     .step();
	}
};

class sensorimotor_core {

	bool dir_left;
	bool enabled;

	uint8_t voltage_pwm;

	Sensors          sensors;
	motor_ifx9201sg  motor;

	bool release_mode;
	uint16_t last_position;

public:

	sensorimotor_core()
	: dir_left(true)
	, enabled(false)
	, voltage_pwm(0)
	, sensors()
	, motor()
	, release_mode()
	, last_position(sensors.position.get_value())
	{
		motor.disable();
		motor.set_pwm(voltage_pwm);
	}

	void step() {

		sensors.step();
		const uint16_t value = sensors.position.get_value();

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

	uint16_t get_position        () { return sensors.position        .get_value(); }
	uint16_t get_current         () { return sensors.current         .get_value(); }
	uint16_t get_voltage_back_emf() { return sensors.voltage_back_emf.get_value(); }
	uint16_t get_voltage_supply  () { return sensors.voltage_supply  .get_value(); }
	uint16_t get_temperature     () { return sensors.temperature     .get_value(); }
};

} /* namespace supreme */
