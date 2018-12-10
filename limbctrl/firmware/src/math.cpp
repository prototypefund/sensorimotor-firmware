/*---------------------------------+
 | Supreme Machines                |
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | December 2018                   |
 +---------------------------------*/

#include <src/math.hpp>

namespace supreme {

static constexpr float Fmax = 32767.f;

float clip(float x) { return clip(x, 1.f); }

float sc_to_float(scdata_t s)
{
	const float value = (s & 0x7fff) / Fmax;
	const float sign = (s >> 15) ? -1.f : 1.f;
	return value * sign;
}

pwm_t sc_to_pwm(scdata_t s)
{
	return { (uint8_t) (s >> 7), (bool) (s & 0x8000) };
}

scdata_t float_to_sc(float f)
{
	scdata_t value = (scdata_t) (clip(std::abs(f)) * Fmax);
	if (f < 0.f)
		value |= 0x8000;
	return value;
}

} /* namespace supreme */
