#ifndef SUPREME_MOTOR_CTRL_TRAPEZOID_H
#define SUPREME_MOTOR_CTRL_TRAPEZOID_H

#include <mbed.h>
#include <halfbridge.h>
#include <brushless_dc.h>
#include <state_lut.h>
#include <hall_sensor.h>
#include <modules.h>

namespace supreme { namespace motor_ctrl {


inline output_t operator*(output_t a, int b) { return static_cast<output_t>(a * b); }

class trapezoid {
public:
    Serial& msg;

    int direction;

    HallSensor3Elements sensor;
    BrushlessDC         motor;
    unsigned            state;

    trapezoid(Serial& msg)
    : msg(msg)
    , direction(0)
    , sensor(D11, D12, D13)
    , motor(100 /* 10kHz */)
    , state()
    {
        sensor.reg_events(this, &trapezoid::update);
        sensor.read();
        state = sensor.get_state();
        set_velocity(.0);
        set_mode();
    }

    /* this method is used as a interrupt service routine
     * and must be kept as short as possible. */
    void update() {
        sensor.read();
        state = sensor.get_state();
        set_mode();
    }

    void set_mode() {
        motor.set_mode( lut::ctrl[state][0]/* * direction*/
                      , lut::ctrl[state][1]/* * direction*/
                      , lut::ctrl[state][2]/* * direction*/ );
    }

    // TODO consider get_mode() or integrate velocity sensor to motor?

    void set_velocity(float duty_cycle) {
        direction = tools::sgn(duty_cycle);
        motor.set_duty_cycle(duty_cycle);
    }

    void step() {
        set_mode();
    }
};



}} // namespace supreme::motor_ctrl

#endif // SUPREME_MOTOR_CTRL_TRAPEZOID
