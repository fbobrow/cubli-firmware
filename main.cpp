#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX);
AttitudeEstimator att_est(250);

// MATLAB comand
char command;

// Ticker
Ticker tic_est;
Ticker tic_blink;
Timer tim;

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

float dt;

// Main program
int main()
{
    pc.baud(230400);  
    att_est.init();
    tic_est.attach(&callback_est, 1.0/250);
    tic_blink.attach(&callback_blink, 0.5);
    tim.start();
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
            tim.reset();
            att_est.estimate();
            dt = tim.read();
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                //pc.printf("%f,%f,%f,%f\n",att_est.q(1,1),att_est.q(2,1),att_est.q(3,1),att_est.q(4,1));
                pc.printf("%.2fms\n",dt*1000.0f);
            }
        }
    }
}