#ifndef SUPREME_SENSORIMOTOR_CORE_HPP
#define SUPREME_SENSORIMOTOR_CORE_HPP

#include <xpcc/architecture/platform.hpp>

#include <motor_ifx9201sg.hpp>
#include <adc.hpp>
#include <temperature.hpp>
#include <common/lowpass.hpp>

namespace supreme {

namespace defaults {
	const uint8_t pwm_limit = 32; /* 12,5% duty cycle */
}

class Sensors {
public:
	uint16_t position, position_0;
	 int16_t velocity;
	uint16_t current;
	uint16_t voltage_back_emf;
	uint16_t voltage_supply;
	uint16_t temperature;

	Sensors()
	: lowpass(/*initial_value=*/.0, /*coeff=*/0.1)
	{}

	void step(void)
	{
		position         = lowpass.step( adc::result[adc::position] << 6 ); /* promote to upper bits and lowpass-filter */
		current          = adc::result[adc::current];
		voltage_back_emf = adc::result[adc::voltage_back_emf];
		voltage_supply   = adc::result[adc::voltage_supply];
		temperature      = get_temperature_celsius(adc::result[adc::temperature]);

		/* increment dt for velocity averaging.
		   count time steps up to 1000ms.
		*/
		dt += (dt < 1000) ? 1 : 0;
	}

	/* get averaged velocity and restart averaging */
	uint16_t restart_velocity_sampling(void) {
		const auto dp = (int32_t) position - position_0; // calculate difference
		position_0 = position;                           // keep last position value
		velocity = ((dp * 1000) / dt) >> 2;              // v = dp/dt with [dt] = ms
		dt = 0;                                          // reset time delta
		return velocity;
	}

private:
	Lowpass<uint16_t> lowpass;
	uint16_t dt = 0;
};

class sensorimotor_core {

	bool enabled;

	struct {
		uint8_t pwm;
		bool    dir;
	} target;

	Sensors          sensors;
	motor_ifx9201sg  motor;

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
