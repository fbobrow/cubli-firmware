#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Motor motor(M1_ENABLE,M1_CURRENT);
Ticker tic_blink;

// Interrupt flags and callback functions
bool flag_blink = false;
void callback_blink() { flag_blink = true; }

// Serial commands
char command;
float i, tau;

// Main program
int main() 
{
    // Initializations
    tic_blink.attach_us(&callback_blink, dt_blink_us);
    // Endless loop
    while (true) 
    {
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (pc.readable()) 
        {
            command = pc.getc();
            if (command == 'r') 
            {
                pc.printf("Ready!\n");
            }
            else if (command == 'c') 
            {
                pc.printf("Current [A]: ");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&i);
                pc.printf("%.1f\n",i);
                motor.set_current(i);
            }
            else if (command == 't') 
            {
                pc.printf("Torque [N.m]: ");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&tau);
                pc.printf("%.3f\n",tau);
                motor.set_torque(tau);
            }
            else if (command == ' ') 
            {
                pc.printf("Aborting...\n\n");
                i = 0.0;
                tau = 0.0;
                motor.set_current(0.0);
            }
        }
    }
}