#ifndef SUPREME_LIMBCTRL_COMMON_HPP
#define SUPREME_LIMBCTRL_COMMON_HPP

#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {

	void error_state(void);
	void assert(bool condition, uint8_t code);

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_COMMON_HPP */
