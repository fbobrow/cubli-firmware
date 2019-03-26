#include "mbed.h"
#include "AttitudeEstimator.h"

DigitalOut led(LED1);
AttitudeEstimator att_est;
Serial pc(SERIAL_TX, SERIAL_RX);

// MATLAB comand
char command;

Timer tim;
Ticker tic_est;
Ticker tic_print;

bool flag_est = false;
bool flag_print = false;

void callback_est()
{
    flag_est = true;
}
void callback_print()
{
    flag_print = true;
}

int main()
{
    pc.baud(230400);  
    att_est.init();
    tic_est.attach(&callback_est, 0.01);
    tic_print.attach(&callback_print, 0.5);

    tim.start();
    float dt_est = 0.0;
    float dt_print = 0.0;
    
    while (true) 
    {
        if (flag_est) 
        {
            flag_est = false;
            tim.reset();
            att_est.estimate();
            dt_est = tim.read();
        }
        if (pc.readable()) {
            command = pc.getc();
            //pc.scanf("%d",&command);
            if (command == 'p') {
                tim.reset();
                pc.printf("%f,%f,%f,%f\n",att_est.q(1,1),att_est.q(2,1),att_est.q(3,1),att_est.q(4,1));
                //pc.printf("%f,%f,%f\n",att_est.omega(1,1),att_est.omega(2,1),att_est.omega(3,1));
                pc.printf("%f,%f\n",dt_est*1000.0,dt_print*1000.0);
                dt_print = tim.read();
            }
        }
        if (flag_print)
        {
            flag_print = false;
            led = !led;
        }
    }
}