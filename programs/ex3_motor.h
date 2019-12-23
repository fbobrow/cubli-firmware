#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
Motor motor(M1_ENABLE,M1_CURRENT);
Ticker tic;

// Interrupt flag and callback functions
bool flag = false;
void callback() { flag = true; }

// Serial commands
char command;
float i, tau;

// Main program
int main() 
{
    // Initializations
    tic.attach_us(&callback, 1e6);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
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
                pc.printf("Current (A): ");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&i);
                pc.printf("%.2f\n",i);
                motor.set_current(i);
            }
            else if (command == 't') 
            {
                pc.printf("Torque (N.m): ");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&tau);
                pc.printf("%.2f\n",tau);
                motor.set_torque(tau);
            }
            else if (command == ' ') 
            {
                pc.printf("Aborting...\n\n");
                motor.set_current(0.0);
            }
        }
    }
}