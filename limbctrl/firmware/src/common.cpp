#include <src/common.hpp>

namespace supreme {

void error_state(void)
{
	led_red::set();
	led_ylw::reset();
	while(1) {
		led_red::toggle();
		led_ylw::toggle();
		xpcc::delayMilliseconds(500);
	}
}

} /* namespace supreme */
