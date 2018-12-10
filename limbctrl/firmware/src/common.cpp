/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#include <src/common.hpp>

namespace supreme {

void blink(uint8_t code) {
	for (uint8_t i = 0; i < 8; ++i)
	{
		if ((code & (0x1 << i)) == 0)
		{
			led_red::reset();
			led_ylw::set();
		} else {
			led_red::set();
			led_ylw::reset();
		}
		xpcc::delayMilliseconds(250);
		led_red::reset();
		led_ylw::reset();
		xpcc::delayMilliseconds(250);
	}
	led_red::reset();
	led_ylw::reset();
}


void assert(bool condition, uint8_t code) {
	if (condition) return;
	led_red::reset();
	led_ylw::reset();
	while(1) {
		blink(code);
		xpcc::delayMilliseconds(1000);
	}
}

void error_state() {
	assert(false, 0);
}

} /* namespace supreme */
