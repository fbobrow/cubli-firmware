#include "mbed.h"
#include "AttitudeEstimator.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX);
AttitudeEstimator att_est;

// MATLAB comand
char command;

// Ticker
Ticker tic_est;
Ticker tic_blink;

// Flags
bool flag_est = false;
bool flag_blink = false;


// Callbacks
void callback_est()
{
    flag_est = true;
}
void callback_blink()
{
    flag_blink = true;
}

// Main program
int main()
{
    pc.baud(230400);  
    att_est.init();
    tic_est.attach(&callback_est, 0.005);
    tic_blink.attach(&callback_blink, 0.5);
    while (true) 
    {
        if (flag_blink)
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_est) 
        {
            flag_est = false;
            att_est.estimate();
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%f,%f,%f,%f\n",att_est.q(1,1),att_est.q(2,1),att_est.q(3,1),att_est.q(4,1));
            }
        }
    }
}