#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
AnalogIn analog(A6);
Escon m1(M1_ENABLE,M1_SPEED,M1_CURRENT);
Escon m2(M2_ENABLE,M2_SPEED,M2_CURRENT);
Escon m3(M3_ENABLE,M3_SPEED,M3_CURRENT);
SpeedEstimator spe_est(freq_estimator);

// MATLAB comand
char command;

// Ticker
Ticker tic_blink;
Ticker tic_motors;

// Flags
bool flag_blink = false;
bool flag_motors = false;
bool flag_spe_est = false;

// Callbacks
void callback_blink()
{
    flag_blink = true;
}
void callback_motors()
{
    flag_motors = true;
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
    tic_motors.attach(&callback_motors, 1.0/freq_estimator);
    tim.start();
    while (true) 
    {
        if (flag_blink)
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_motors)
        {
            flag_motors = false;
            flag_spe_est = true;
        }
        if (flag_spe_est) 
        {
            flag_spe_est = false;
            analog.read();
            spe_est.update(m1.read_speed(),m2.read_speed(),m3.read_speed());
            pc.printf("%6.2f\n",spe_est.omega(1,1));
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 't') {
                pc.printf("Hello world\n");
            }
            else if (command == 's') {
                pc.printf("Speed (rad/s): %6.2f | %6.2f | %6.2f\n",spe_est.omega(1,1),spe_est.omega(2,1),spe_est.omega(3,1));
            }
            else if (command == 'a') {
                m1.set_current(0.0f);
                m2.set_current(0.0f);
                m3.set_current(0.0f);
            }
            else if (command == 'c') {
                pc.printf("Current (A): \n");
                while (!pc.readable())
                {
                }
                pc.scanf("%f",&i);
                m1.set_current(i);
                //m2.set_current(i);
                //m3.set_current(i);
            }
        }
    }
}