#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
AttitudeEstimator2 att_est(IMU_SDA,IMU_SCL);
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }

// Serial commands
char command;

Timer tim;
float delta_t;

// Main program
int main() 
{
    // Initializations
    tim.start();
    att_est.init();
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, dt_blink_us);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
            delta_t = tim.read();
            tim.reset();
            att_est.estimate();
        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%.4f,%.4f,%.4f,%.4f\n",att_est.q(1,1),att_est.q(2,1),att_est.q(3,1),att_est.q(4,1));
                pc.printf("%.4f,%.4f,%.4f\n",att_est.omega(1,1),att_est.omega(2,1),att_est.omega(3,1));
                pc.printf("%.4f\n",delta_t);
                pc.printf("%.10f,%.10f,%.10f,%.10f,%.10f,%.10f,%.10f\n",att_est.P(1,1),att_est.P(2,2),att_est.P(3,3),att_est.P(4,4),att_est.P(5,5),att_est.P(6,6),att_est.P(7,7));
            }
        }
    }
}