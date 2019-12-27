#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
AHRS ahrs;
Ticker tic, tic_blink;

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
    ahrs.init();  
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, 1e6);
    // Endless loop
    while (true) 
    {
        if (flag)
        {
            flag = false;
            ahrs.update();
        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%f,%f,%f,%f\n",ahrs.q(1,1),ahrs.q(2,1),ahrs.q(3,1),ahrs.q(4,1));
            }
        }
    }
}