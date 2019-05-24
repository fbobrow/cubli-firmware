#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);

DigitalOut m1_en(M2_EN);
AnalogIn   m1_speed(M2_SPEED);
PwmOut     m1_current(M2_CURRENT); 

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
    m1_current.period_ms(2);
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
                //pc.printf("Speed (rpm): %.1f\n",m1_speed.read()*27036.45-19833.35);
                pc.printf("Speed (rpm): %.1f\n",m1_speed.read()*28674.49-18381.31);
                //pc.printf("Speed (float): %.4f\n",m1_speed.read());
            }
            else if (command == 'f') {
                m1_en = true;
                m1_current = 1.0f;
            }
            else if (command == 'h') {
                m1_en = true;
                m1_current = 0.5f;
            }
            else if (command == 'z') {
                m1_en = true;
                m1_current = 0.0f;
            }
            else if (command == 'd') {
                m1_en = false;
                m1_current = 0.0f;
            }
        }
    }
}