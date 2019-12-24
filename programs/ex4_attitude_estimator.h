#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
AttitudeEstimator att_est;
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

// Main program
int main() 
{
    // Initializations
    att_est.init();
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, 1e6);
    tic_print.attach_us(&callback_print, 1e4);
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
        if (flag_print) 
        {
            flag_print = false;
            pc.printf("%.4f\t%.4f\n",att_est.theta_s_m,att_est.theta_s);
        }
    }
}