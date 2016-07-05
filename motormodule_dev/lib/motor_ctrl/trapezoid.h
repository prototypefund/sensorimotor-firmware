#ifndef SUPREME_MOTOR_CTRL_TRAPEZOID_H
#define SUPREME_MOTOR_CTRL_TRAPEZOID_H

#include <mbed.h>
#include <halfbridge.h>
#include <brushless_dc.h>

namespace supreme { namespace motor_ctrl {

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

namespace lut {
    const output_t ctrl[7][3] = { {flt, gnd, pwm} /* normal cases, 0..5 */
                                , {pwm, gnd, flt}
                                , {pwm, flt, gnd}
                                , {flt, pwm, gnd}
                                , {gnd, pwm, flt}
                                , {gnd, flt, pwm}
                                /* set all floating in case of fault */
                                , {flt, flt, flt} };

    const unsigned stat[6][3] = { { 1, 0, 1 }
                                , { 1, 0, 0 }
                                , { 1, 1, 0 }
                                , { 0, 1, 0 }
                                , { 0, 1, 1 }
                                , { 0, 0, 1 } };
}

class hall_sensor_3e {
public:

    DigitalIn hall_1, // latching hall sensors
              hall_2,
              hall_3;

    struct bin_state_t {
        unsigned h1, h2, h3; // mirror state of hall sensors
    } sensor;

    hall_sensor_3e(PinName pin_1, PinName pin_2, PinName pin_3)
    : hall_1(pin_1)
    , hall_2(pin_2)
    , hall_3(pin_3)
    {
        hall_1.mode(PullUp);
        hall_2.mode(PullUp);
        hall_3.mode(PullUp);
    }

    void read() {
        sensor.h1 = hall_1.read(); //TODO use interrupts
        sensor.h2 = hall_2.read();
        sensor.h3 = hall_3.read();
    }

    unsigned get_state() const {

        for (unsigned i = 0; i < 6; ++i)
            if (sensor.h1 == lut::stat[i][0]
            and sensor.h2 == lut::stat[i][1]
            and sensor.h3 == lut::stat[i][2]) return i;
        return 6; // fault case
    }

    const bin_state_t& get_binary_state() const { return sensor; }
};

class trapezoid {
public:
    Serial& msg;

    enum direction_t {
        forwards = 0,
        backwards = 1
    } dir;

    hall_sensor_3e sensor;
    brushless_dc   motor;

    trapezoid(Serial& msg)
    : msg(msg)
    , dir(forwards)
    , sensor(D11, D12, D13)
    , motor(1000 /* 1kHz */) {}

    void step() {

        sensor.read();
        unsigned state = sensor.get_state();

        //if (dir == forwards)
        motor.set_mode( lut::ctrl[state][0]
                      , lut::ctrl[state][1]
                      , lut::ctrl[state][2] );
        //else TODO other direction
    }
};

}} // namespace supreme::motor_ctrl

#endif // SUPREME_MOTOR_CTRL_TRAPEZOID
