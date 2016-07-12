#ifndef SUPREME_MOTOR_CTRL_STATE_LUT_H
#define SUPREME_MOTOR_CTRL_STATE_LUT_H

#include <halfbridge.h>

// clockwise
/*--+----------+-------------+
| # | H3 H2 H1 |  W   V   U  |
+---+----------+-------------+
| 0 |  1  0  1 | pwm gnd flt |
| 1 |  1  0  0 | pwm flt gnd |
| 2 |  1  1  0 | flt pwm gnd |
| 3 |  0  1  0 | gnd pwm flt |
| 4 |  0  1  1 | gnd flt pwm |
| 5 |  0  0  1 | flt gnd pwm |
+---+----------+------------*/

namespace supreme { namespace motor_ctrl { namespace lut {

    /* Note: maybe this LUT is specific to the chinese motor being used
     * and must be adapted if another manufacturer comes into play. */
    const output_t ctrl[7][3] = { /* allowed states, 0..5 */
                                  {pwm, gnd, flt}
                                , {pwm, flt, gnd}
                                , {flt, pwm, gnd}
                                , {gnd, pwm, flt}
                                , {gnd, flt, pwm}
                                , {flt, gnd, pwm}
                                /* set all floating in case of fault */
                                , {flt, flt, flt} };

    const unsigned stat[6][3] = { { 1, 0, 1 }
                                , { 1, 0, 0 }
                                , { 1, 1, 0 }
                                , { 0, 1, 0 }
                                , { 0, 1, 1 }
                                , { 0, 0, 1 } };

}}} // namespace supreme::motor_ctrl::lut

#endif // SUPREME_MOTOR_CTRL_STATE_LUT_H
