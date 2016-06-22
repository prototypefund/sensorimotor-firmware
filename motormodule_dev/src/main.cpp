#include "mbed.h"
#include "Serial.h"

Serial pc(USBTX, USBRX);
PwmOut myled(LED1);

/* motor drive pwm output pins */
PwmOut u_in   (D2);
PwmOut u_inhib(D3);
PwmOut v_in   (D4);
PwmOut v_inhib(D5);
PwmOut w_in   (D6);
PwmOut w_inhib(D7);

/* voltage + current sensors */
AnalogIn u_us(A0);
// AnalogIn u_is(A1);
AnalogIn v_us(A2);
// AnalogIn v_is(A3);
AnalogIn w_us(A4);
// AnalogIn w_is(A5);

/* latching hall sensors */
// DigitalIn hall_1(D11);
// DigitalIn hall_2(D12);
// DigitalIn hall_3(D13);

int main() {
    u_inhib = 1;
    v_inhib = 1;
    w_inhib = 1;

    u_in.period_us(1000);
    v_in.period_us(1000);
    w_in.period_us(1000);

    u_in = 0;
    v_in = 0;
    w_in = 0;

    myled = 0.0;
    unsigned state = 0;

    while(1) {
        wait(2.00);
        state = 1 - state;
        u_in = 0.25 + 0.5 * state;
        v_in = 0.25 + 0.5 * state;
        w_in = 0.25 + 0.5 * state;

        // float u = u_us.read();
        // float v = v_us.read();
        // float w = w_us.read();

        myled = state;
        //pc.printf("u=%1.2fV v=%1.2fV w=%1.2fV \n", u, v, w_us);
    }
}
