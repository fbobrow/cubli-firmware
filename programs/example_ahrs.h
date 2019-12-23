#include "mbed.h"
#include "src/modules/submodules/ahrs.h"

// Objects
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
AHRS ahrs;

// MATLAB comand
char command;

// Ticker
Ticker tic;

// Flags
bool flag = false;

// Callbacks
void callback()
{
    flag = true;
}

// Main program
int main()
{
    ahrs.init();  
    tic.attach_us(&callback, dt_us);
    while (true) 
    {
        if (flag)
        {
            flag = false;
            ahrs.update();
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%f,%f,%f,%f\n",ahrs.q(1,1),ahrs.q(2,1),ahrs.q(3,1),ahrs.q(4,1));
                //pc.printf("%f,%f,%f,%f,%f,%f\n",ahrs.a(1,1),ahrs.a(2,1),ahrs.a(3,1),ahrs.m(1,1),ahrs.m(2,1),ahrs.m(3,1));
            }
        }
    }
}