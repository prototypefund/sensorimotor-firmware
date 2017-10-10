/*

   +----------------------------+
   | 2017, Supreme Machines GbR |
   | Sensorimotor Firmware      |
   +----------------------------+

*/

#include <xpcc/architecture/platform.hpp>
#include <sensorimotor_core.hpp>
#include <communication.hpp>


/* this is called once TCNT0 = OCR0A = 249 *
 * resulting in a 1 ms cycle time, 1kHz    */

volatile unsigned int cycles = 0;
volatile bool timer_state = false;

ISR (TIMER0_COMPA_vect)
{
	++cycles;
	if (cycles > 9)
		cycles = 0;
	timer_state = !timer_state;
}

/*
ISR ( USART_RX_vect )
{
	com.step_irq();
}
*/

int main()
{
	Board::initialize();
	led::yellow::setOutput();
	led::red::setOutput();

	supreme::sensorimotor_core ux;

	/* Design of the 1kHz/100Hz main loop:
	 * 16Mhz clock, prescaler 64 -> 16.000.000 / 64 = 250.000 increments per second
	 * diveded by 1000 -> 250 increments per ms
	 * hence, timer compare register to 250-1 -> ISR inc ms counter -> modulo 10 -> 100Hz loop
	 *
	 * configure timer 0:
	 */
	TCCR0A = (1<<WGM01);             // CTC mode
	TCCR0B = (1<<CS01) | (1<<CS00);  // set prescaler to 64
	OCR0A = 249;                     // set timer compare register to 250-1
	TIMSK0 = (1<<OCIE0A);            // enable compare interrupt

	unsigned long total_cycles = 0;

	supreme::communication_ctrl com(ux);

	bool old_timer_state = false;

	while(1) /* main loop */
	{
		while (cycles > 0) {
			if (timer_state != old_timer_state) {
				com.step();
				old_timer_state = timer_state;
			}
		};     // wait until cycles == 0
		// consider using xpcc::delayNanoseconds(delayTime);
		led::red::set();   // red led on, begin of cycle
		ux.step();

		++total_cycles;
		led::red::reset(); // red led off, end of cycle
		while (cycles == 0);    // eat up rest of the time
	}
	return 0;
}
