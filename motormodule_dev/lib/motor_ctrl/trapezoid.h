#ifndef SUPREME_MOTOR_CTRL_TRAPEZOID_H
#define SUPREME_MOTOR_CTRL_TRAPEZOID_H

#include <mbed.h>
#include <halfbridge.h>
#include <brushless_dc.h>
#include <state_lut.h>
#include <hall_sensor.h>

namespace supreme { namespace motor_ctrl {

class trapezoid {
public:
    Serial& msg;

    enum direction_t {
        backwards = -1,
        none      =  0,
        forwards  = +1
    } dir;

    HallSensor3Elements sensor;
    BrushlessDC         motor;

    trapezoid(Serial& msg)
    : msg(msg)
    , dir(forwards)
    , sensor(D11, D12, D13)
    , motor(100 /* 10kHz */)
    {
        sensor.reg_events(this, &trapezoid::update);
        sensor.read();
        motor.set_duty_cycle(0.10);
    }

    /* this method is used as a interrupt service routine
     * and must be kept as short as possible. */
    void update() {
        sensor.read();
        unsigned state = sensor.get_state();

        motor.set_mode( static_cast<output_t>(lut::ctrl[state][0] * dir)
                      , static_cast<output_t>(lut::ctrl[state][1] * dir)
                      , static_cast<output_t>(lut::ctrl[state][2] * dir) );

        //motor.set_duty_cycle(0.10);
        //msg.printf("updated\n");
    }

    void step() { /* TODO implement */ }
};

}} // namespace supreme::motor_ctrl

#endif // SUPREME_MOTOR_CTRL_TRAPEZOID
