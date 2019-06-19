#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
Escon m1(M2_EN,M2_SPEED,M2_CURRENT);

// MATLAB comand
char command;

// Ticker
Ticker tic_blink;

// Flags
bool flag_blink = false;

// Callbacks
void callback_blink()
{
    flag_blink = true;
}

// Debug timer
Timer tim;
float dt;

float i;

float speed;

// Main program
int main()
{
    tic_blink.attach(&callback_blink, 1.0/freq_blink);
    tim.start();
    while (true) 
    {
        if (flag_blink)
        {
            flag_blink = false;
            led = !led;
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 't') {
                pc.printf("Hello world\n");
            }
            else if (command == 's') {
                speed = m1.read();
                pc.printf("Speed (rpm): %.3f\n",speed);
            }
            else if (command == 'a') {
                m1.set(0.0f);
            }
            else if (command == 'c') {
                pc.printf("Current (A): \n");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&i);
                m1.set(i);
            }
        }
    }
}