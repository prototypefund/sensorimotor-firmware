/*

   +----------------------------+
   | 2017, Supreme Machines GbR |
   | Sensorimotor Firmware      |
   +----------------------------+

*/

#include <xpcc/architecture/platform.hpp>
#include <sensorimotor_core.hpp>


/* this is called once TCNT0 = OCR0A = 249 *
 * resulting in a 1 ms cycle time, 1kHz    */

volatile unsigned int cycles = 0;

ISR (TIMER0_COMPA_vect)
{
	++cycles;
	if (cycles > 9)
		cycles = 0;
}


int main()
{
	Board::initialize();
	led_D5::setOutput();
	supreme::sensorimotor_core ux;

	/* configure timer 0 */
	TCCR0A = (1<<WGM01);             // CTC mode
	TCCR0B = (1<<CS01) | (1<<CS00);  // set prescaler to 64
	OCR0A = 249;
	TIMSK0 = (1<<OCIE0A);            // enable compare interrupt


	unsigned long total_cycles = 0;

	while(1) /* main loop */
	{
		while (cycles > 0); // wait until cycles == 0
		Board::led_D5::set();   // green led on
		ux.step();

		if (total_cycles % 100 == 0)
			serialStream /*<< "\r"*/ << total_cycles << "\n\r";

		/* design of the proper 1kHz control loop + 100Hz com loop:
		 * 16Mhz clock, prescaler 64 -> 16.000.000 / 64 = 250.000 increments per second
		 * diveded by 1000 -> 250 increments per ms
		 * hence, timer compare register to 250 -> ISR inc ms counter -> modulo 10 -> 100Hz com loop
		 */

		 ++total_cycles;
		 Board::led_D5::reset(); // green led off
		 while (cycles == 0); // eat up rest of the time
	}
	return 0;
}
