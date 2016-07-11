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
        forwards = 0,
        backwards = 1
    } dir;

    HallSensor3Elements sensor;
    brushless_dc   motor;

    trapezoid(Serial& msg)
    : msg(msg)
    , dir(forwards)
    , sensor(D11, D12, D13)
    , motor(100 /* 1kHz */)
    {
        sensor.reg_events(this, &trapezoid::update);
        sensor.read();
    }

    /* this method is used as a interrupt service routine
     * and must be kept as short as possible. */
    void update() {
        sensor.read();
        unsigned state = sensor.get_state();

        //if (dir == forwards)
        motor.set_mode( lut::ctrl[state][0]
                      , lut::ctrl[state][1]
                      , lut::ctrl[state][2] );
        //else TODO other direction
        //msg.printf("updated\n");
    }

    void step() { /* TODO implement */ }

};

}} // namespace supreme::motor_ctrl

#endif // SUPREME_MOTOR_CTRL_TRAPEZOID
