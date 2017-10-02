/*

   +----------------------------+
   | 2017, Supreme Machines GbR |
   | Sensorimotor               |
   +----------------------------+

*/

/*
   Check serial output with minicom
   $ minicom -D /dev/ttyUSB1 -b 1000000
*/

#include <xpcc/architecture/platform.hpp>
#include <../firmware/motor_ifx9201sg.hpp>

using namespace xpcc::atmega;
typedef xpcc::avr::SystemClock clock;

int main()
{
	initialize();

	led::yellow::setOutput(); // yellow
	led::red::setOutput(); // red
	D3::setOutput();     // drive enable RS485
	D3::reset();

	supreme::motor_ifx9201sg  motor;

	led::red::toggle();
	uint32_t value = 0;

	motor.enable();
	motor.set_pwm(32);

	while(true)
	{
		value++;
		led::yellow::toggle();
		led::red::toggle();
		motor.toggle_direction();

		xpcc::delayMilliseconds(1000);
		rs485::drive_enable::set(); // drive enable RS485
		serialStream << value << "\n\r";//xpcc::endl;
		rs485::drive_enable::reset(); // drive disable RS485

	}
	return 0;
}
