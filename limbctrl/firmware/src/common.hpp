/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 +---------------------------------*/

#ifndef SUPREME_LIMBCTRL_COMMON_HPP
#define SUPREME_LIMBCTRL_COMMON_HPP

#include <boards/limbctrl_f411re.hpp>
#include <xpcc/architecture/platform.hpp>

using namespace Board;

namespace supreme {

	void error_state(void);
	void assert(bool condition, uint8_t code);

} /* namespace supreme */

#endif /* SUPREME_LIMBCTRL_COMMON_HPP */
