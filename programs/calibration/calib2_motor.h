#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
AnalogIn analog(A6);
Escon motor_1(M1_ENABLE,M1_SPEED,M1_CURRENT);

// MATLAB comand
char command;

// Ticker
Ticker tic_blink;
Ticker tic_print;

// Flags
bool flag_blink = false;
bool flag_print = false;

// Callbacks
void callback_blink()
{
    flag_blink = true;
}
void callback_print()
{
    flag_print = true;
}

// Debug timer
Timer tim;
float dt;

float tau;

// Main program
int main()
{
    analog.read();
    tic_blink.attach(&callback_blink, 1.0/freq_blink);
    tic_print.attach(&callback_print, 0.01);
    tim.start();
    while (true) 
    {
        if (flag_blink)
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_print) 
        {
            flag_print = false;
            motor_1.read();
            pc.printf("%6.4f\t%6.4f\n",tau,motor_1.omega-b_omega_1);
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'h') {
                pc.printf("Hello world\n");
            }
            else if (command == 'a') {
                tau = 0.0f;
                motor_1.set_current(tau);
            }
            else if (command == 't') {
                //pc.printf("\n>> Torque (Nm): \n");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&tau);
                motor_1.set_torque(tau);
            }
        }
    }
}