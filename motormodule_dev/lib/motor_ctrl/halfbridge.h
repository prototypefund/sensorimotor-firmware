#ifndef SUPREME_MOTOR_CTRL_HALFBRIDGE_H
#define SUPREME_MOTOR_CTRL_HALFBRIDGE_H

#include <mbed.h>

namespace supreme { namespace motor_ctrl {

enum output_t {
    gnd = -1,
    flt =  0,
    pwm = +1
};

struct halfbridge
{
    typedef mbed::PwmOut pwm_t;

    pwm_t input;
    pwm_t enable;
    float duty_cycle;

    halfbridge(PinName pin_input, PinName pin_enable, const unsigned period_us)
    : input(pin_input)
    , enable(pin_enable)
    , duty_cycle(0.40)
    {
        enable = .0;
        input  = .0;
        enable.period_us(period_us);
        input .period_us(period_us);
    }

    void set_mode(const output_t mode) {
        switch(mode) {
        case pwm:
            enable = 1;
            input = duty_cycle;
            break;
        case gnd:
            enable = 1;
            input = .0;
            break;
        case flt:
        default:
            enable = 0;
            input = .0;
            break;
        }
    };

};

}} // namespace supreme::motor_ctrl

#endif // SUPREME_MOTOR_CTRL_HALFBRIDGE_H
