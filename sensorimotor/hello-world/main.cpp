/*

   +----------------------------+
   | 2017, Supreme Machines GbR |
   | Sensorimotor               |
   +----------------------------+

*/

#include <xpcc/architecture/platform.hpp>

using namespace xpcc::atmega;
typedef xpcc::avr::SystemClock clock;

const unsigned lower_bound = 150;
const unsigned upper_bound = 900;

int main()
{
	Board::initialize();
	led_D5::setOutput();
	//led_D5::set();

	motor::VSO::set(); // enable motor bridge logic
	motor::DIR::set(); // set direction
	motor::PWM::set(); // set full speed


	/* setup pwm */

	TCCR1A = (1<<WGM10)|(1<<COM1A1); // Set up the two Control registers of Timer1.
	TCCR1B = (1<<WGM12)              // Wave Form Generation is Fast PWM 8 Bit,
	      // | (1<<CS12);               // OC1A and OC1B are cleared on compare match
	      // | (1<<CS10);
	         | (1<<CS11);            // set prescaler to 8 -> 7812,5 Hz

	OCR1A = 32;


	A0::setInput();
	Adc::initialize<clock, 115000>();
	Adc::setReference(Adc::Reference::InternalVcc);

	uint16_t value = Adc::readChannel(7);



	Adc::setChannel(7);
	Adc::startConversion();

	bool dir_left = true;

	while (1)
	{
		//Board::led_D5::toggle();
		//motor::DIR::toggle();
		/*motor::PWM::set();
		xpcc::delayMilliseconds(1);
		motor::PWM::reset();*/
		xpcc::delayMilliseconds(10);


		if (Adc::isConversionFinished())
		{
			value = Adc::getValue();
			if (dir_left && value < lower_bound) {
				dir_left = false;
				motor::DIR::toggle();
			} else if (!dir_left && value > upper_bound){
				dir_left = true;
				motor::DIR::toggle();
			}



			//OCR1A = value >> 2; // turn to 8 bit
			//xpcc::delayMilliseconds(5);
			//xpcc::delayMilliseconds(5);
			// restart the conversion
			Adc::setChannel(7);
			Adc::startConversion();
		}
		serialStream << value << "\n\r";//xpcc::endl;
	}
	return 0;
}









//		xpcc::delayMilliseconds(Board::Button::read() ? 125 : 100);
//#ifdef XPCC_BOARD_HAS_LOGGER
//		static uint32_t counter(0);
//		XPCC_LOG_INFO << "Loop counter: " << (counter++) << xpcc::endl;
//#endif
