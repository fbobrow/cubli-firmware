#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);

DigitalOut m1_en(M1_EN);
AnalogIn   m1_speed(M1_SPEED);
PwmOut     m1_current(M1_CURRENT); 

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
                pc.printf("Hello world");
            }
            else if (command == 's') {
                pc.printf("Speed (rad/s): %.6f",m1_speed.read());
            }
            else if (command == 'a') {
                m1_en = true;
                m1_current = 0.1f;
            }
            else if (command == 'd') {
                m1_en = false;
                m1_current = 0.0f;
            }
        }
    }
}