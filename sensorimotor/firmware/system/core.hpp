#ifndef SUPREME_SENSORIMOTOR_CORE_HPP
#define SUPREME_SENSORIMOTOR_CORE_HPP

#include <xpcc/architecture/platform.hpp>

#include <system/adc.hpp>
#include <common/temperature.hpp>

namespace supreme {

namespace defaults {
	const uint8_t pwm_limit = 32; /* 12,5% duty cycle */
}

class Sensors {
public:
	uint16_t position         = 0;
	uint16_t current          = 0;
	uint16_t voltage_back_emf = 0;
	uint16_t voltage_supply   = 0;
	uint16_t temperature      = 0;

	Sensors() { init(); }

	void init(void)
	{
		for (uint8_t i = 0; i < 6; ++i)
			f[i] = (int16_t) adc::result[adc::position];
	}

	void step(void)
	{
		position         = adc::result[adc::position] << 6; /* promote to upper bits and lowpass-filter */
		current          = adc::result[adc::current];
		voltage_back_emf = adc::result[adc::voltage_back_emf];
		voltage_supply   = adc::result[adc::voltage_supply];
		temperature      = get_temperature_celsius(adc::result[adc::temperature]);

		/* increment dt for velocity averaging.
		   count time steps up to 1000ms.
		*/
		if (dt < 1000) ++dt;
		f[0] = (int16_t) adc::result[adc::position];
	}

	/* get velocity and restart averaging */
	uint16_t restart_velocity_sampling(void) {
		/* shift velocity regs*/
		f[5] = f[4];
		f[4] = f[3];
		f[3] = f[2];
		f[2] = f[1];
		f[1] = f[0];

		const int16_t velocity = (dt < 1000)
		                       ?
		                         ((int32_t)  f[0] - f[5]
		                              + 3 * (f[1] - f[4])
		                              + 2 * (f[2] - f[3]) ) * 1000 / dt
		                       : 0; // return 0, if time delta is too long.

		dt = 0; // reset time delta
		return velocity;
	}

private:
	uint16_t dt = 1000;
	 int16_t f[6];
};

template <typename MotorDriverType>
class sensorimotor_core {

	bool enabled;

	struct {
		uint8_t pwm;
		bool    dir;
	} target;

	Sensors          sensors;
	MotorDriverType  motor;

	uint8_t          watchcat = 0;
	uint8_t          max_pwm = defaults::pwm_limit;

public:

	sensorimotor_core()
	: enabled(false)
	, target()
	, sensors()
	, motor()
	{
		motor.disable();
		motor.set_pwm(0);
	}

	void apply_target_values(void) {
		if (enabled) {
			motor.set_pwm(target.pwm);
			motor.set_dir(target.dir);
			motor.enable();
		} else {
			motor.set_pwm(0);
			motor.disable();
			target.pwm = 0;
		}
	}

	void init_sensors(void) { sensors.init(); }

	void step(void) {
		apply_target_values();
		sensors.step();

		/* safety switchoff */
		if (watchcat < 100) watchcat++;
		else enabled = false;
	}

	void set_pwm_limit (uint8_t lim) { max_pwm = lim; }
	void set_target_pwm(uint8_t pwm) { target.pwm = pwm < max_pwm ? pwm : max_pwm; }
	void set_target_dir(bool    dir) { target.dir = dir; }

	void enable()  { enabled = true; watchcat = 0; }
	void disable() { enabled = false; }
	bool is_enabled() const { return enabled; }

	uint16_t get_position        () const { return sensors.position; }
	uint16_t get_velocity        ()       { return sensors.restart_velocity_sampling(); }
	uint16_t get_current         () const { return sensors.current; }
	uint16_t get_voltage_back_emf() const { return sensors.voltage_back_emf; }
	uint16_t get_voltage_supply  () const { return sensors.voltage_supply; }
	uint16_t get_temperature     () const { return sensors.temperature; }
};

} /* namespace supreme */

#endif /* SUPREME_SENSORIMOTOR_CORE_HPP */
