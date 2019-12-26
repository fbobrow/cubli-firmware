#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
Motor motor;
WheelEstimator whe_est;
AttitudeEstimator att_est;
Controller cont;
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

//
float theta_s_r = 45.0*pi/180;
float theta_s, omega_s, theta_w, omega_w;
float tau;

// Main program
int main() 
{
    // Initializations
    whe_est.init();
    att_est.init();  
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, dt_blink_us);
    tic_print.attach_us(&callback_print, dt_print_us);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
            whe_est.estimate(tau);
            att_est.estimate(tau,whe_est.omega_w);
            theta_s = pi/4-att_est.theta_s;
            omega_s = -att_est.omega_s;
            theta_w = -whe_est.theta_w;
            omega_w = -whe_est.omega_w;
            cont.control(theta_s, omega_s, theta_w, omega_w);

            if (abs(theta_s) <= 5.0*pi/180.0)
            {
                tau = -cont.tau;
            }
            else 
            {
                tau = 0.0;
            }

            motor.set_torque(tau);

        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_print) 
        {
            flag_print = false;
            pc.printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\t%8.4f\n",theta_s,omega_s,theta_w,omega_w,tau/Km);
        }
    }
}