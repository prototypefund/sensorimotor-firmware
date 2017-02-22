/*

   +----------------------------+
   | 2017, Supreme Machines GbR |
   | Sensorimotor Firmware      |
   +----------------------------+

*/

#include <xpcc/architecture/platform.hpp>
#include <sensorimotor_core.hpp>

int main()
{
	Board::initialize();
	led_D5::setOutput();
	supreme::sensorimotor_core ux;

	while(1) /* main loop */
	{
		Board::led_D5::set();   // green led on
		ux.step();
		Board::led_D5::reset(); // green led off

		xpcc::delayMilliseconds(10); //TODO: make proper timer for 1kHz loop
	}
	return 0;
}
