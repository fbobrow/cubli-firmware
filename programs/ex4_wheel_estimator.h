#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
Motor motor(M1_ENABLE,M1_CURRENT);
WheelEstimator we(M1_SPEED);
Ticker tic, tic_blink, tic_print;

// Interrupt flag and callback functions
bool flag = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

// Serial commands
char command;
float i, tau;

// Main program
int main() 
{
    // Initializations
    we.init();
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, 1e6);
    tic_print.attach_us(&callback_print, 1e5);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
            we.predict();
            we.correct();
        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_print) 
        {
            flag_print = false;
            pc.printf("%.2f\t%.2f\n",we.omega_m,we.omega);
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