/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#ifndef SUPREME_LIMBCONTROLLER_MATH
#define SUPREME_LIMBCONTROLLER_MATH

#include <cstdint>
#include <limits>
#include <algorithm>

namespace supreme {

/* spinalcord data:
   MSB is sign: 0=pos, 1=neg

   sbbb.bbbb.bbbb.bbbb => 15 bit of data, 0..32767 for absolute values
*/
typedef std::uint16_t scdata_t;

struct pwm_t {
	uint8_t dc;
	bool dir;
};

/* limits float values */
template <typename T>
T clip(T x, T min, T max)
{
	if      (x < min) return min;
	else if (x > max) return max;
	else              return x;
}

template <typename T>
T clip(T x, T lim) { return clip(x, -lim, lim); }

float clip(float x);

/* converts spinalcord data to float [-1,+1] */
float sc_to_float(scdata_t s);

/* limits f to interval [-1,+1] and
   converts float to spinalcord data */
scdata_t float_to_sc(float f);

/* converts a spinalcord value to 8bit and dir pwm type */
pwm_t sc_to_pwm(scdata_t s);


} /* namespace supreme */

#endif /* SUPREME_LIMBCONTROLLER_MATH */
