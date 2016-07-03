#include <mbed.h>
#include <Serial.h>

#include <brushless_dc.h>
#include <trapezoid.h>

namespace supreme {

/* voltage + current sensors */
// AnalogIn u_us(A0);
// AnalogIn u_is(A1);
// AnalogIn v_us(A2);
// AnalogIn v_is(A3);
// AnalogIn w_us(A4);
// AnalogIn w_is(A5);

class main_app {

    motor_ctrl::trapezoid ctrl;
    Serial msg;
    //PwmOut myled;
    unsigned state;

public:
    main_app()
    : ctrl()
    , msg(USBTX, USBRX)
    // , myled(LED1)
    , state(0)
    {
        msg.baud(115200);
        // myled = 0.0;
    }

    void step() {
        ctrl.step();
        wait(0.01);
        state = 1 - state;
        // myled = 1.0 * state;

        // float u = u_us.read();
        // float v = v_us.read();
        // float w = w_us.read();

        motor_ctrl::hall_sensor_3e::bin_state_t st = ctrl.sensor.get_binary_state();
        unsigned pos = ctrl.sensor.get_state();
        //msg.printf("u=%1.2fV v=%1.2fV w=%1.2fV \n", u, v, w_us);
        msg.printf("%u %u %u %u\n", st.h1, st.h2, st.h3, pos);

    } // loop

}; // class main_app

} // namspace supreme

int main() {
    supreme::main_app app;
    while(1)
        app.step();
}
