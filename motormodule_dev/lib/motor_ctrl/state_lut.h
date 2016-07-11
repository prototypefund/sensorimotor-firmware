#ifndef SUPREME_MOTOR_CTRL_STATE_LUT_H
#define SUPREME_MOTOR_CTRL_STATE_LUT_H

#include <halfbridge.h>

// clockwise
/*--+----------+-------------+
| # | H3 H2 H1 |  W   V   U  |
+---+----------+-------------+
| 0 |  1  0  1 | flt gnd pwm |
| 1 |  1  0  0 | pwm gnd flt |
| 2 |  1  1  0 | pwm flt gnd |
| 3 |  0  1  0 | flt pwm gnd |
| 4 |  0  1  1 | gnd pwm flt |
| 5 |  0  0  1 | gnd flt pwm |
+---+----------+------------*/

namespace supreme { namespace motor_ctrl { namespace lut {


    const output_t ctrl[7][3] = { //{flt, gnd, pwm} /* normal cases, 0..5 */
                                /*,*/ {pwm, gnd, flt}
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
