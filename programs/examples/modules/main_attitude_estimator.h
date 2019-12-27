#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
AttitudeEstimator att_est(IMU_SDA,IMU_SCL);
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }

// Serial commands
char command;

// Main program
int main() 
{
    // Initializations
    att_est.init();
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, dt_blink_us);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
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
                pc.printf("%.4f\n",att_est.theta_s);
            }
        }
    }
}