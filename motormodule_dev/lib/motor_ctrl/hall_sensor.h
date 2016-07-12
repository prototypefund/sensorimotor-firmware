#ifndef SUPREME_MOTOR_CTRL_HALL_SENSOR_H
#define SUPREME_MOTOR_CTRL_HALL_SENSOR_H

#include <mbed.h>
#include <state_lut.h>

namespace supreme { namespace motor_ctrl {

class HallSensor {

    DigitalIn sensor;
    InterruptIn event;

public:
    HallSensor(PinName pin)
    : sensor(pin)
    , event(pin)
    {
        sensor.mode(PullUp);
    }

    unsigned read() { return sensor.read(); }

    template<typename T>
    void reg_events(T* tptr, void (T::*mptr)(void)) {
        event.rise(tptr, mptr);
        event.fall(tptr, mptr);
        sensor.mode(PullUp); // must be set again after attaching ISR
    }
};

class HallSensor3Elements {
public:

    HallSensor hall_1, // latching hall sensors
               hall_2,
               hall_3;

    struct bin_state_t {
        unsigned h1, h2, h3; // mirror state of hall sensors
    } sensor;

    HallSensor3Elements(PinName pin_1, PinName pin_2, PinName pin_3)
    : hall_1(pin_1)
    , hall_2(pin_2)
    , hall_3(pin_3)
    {}

    template<typename T>
    void reg_events(T* tptr, void (T::*mptr)(void)) {
        hall_1.reg_events(tptr, mptr);
        hall_2.reg_events(tptr, mptr);
        hall_3.reg_events(tptr, mptr);
    }

    void read() {
        sensor.h1 = hall_1.read();
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

}} // namespace supreme::motor_ctrl

#endif // SUPREME_MOTOR_CTRL_HALL_SENSOR_H
