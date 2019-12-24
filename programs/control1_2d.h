#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
Motor motor(M1_ENABLE,M1_CURRENT);
WheelEstimator we(M1_SPEED);
AttitudeEstimator att_est;
Ticker tic, tic_blink, tic_print;

FeedbackLinearization fbl;
LinearQuadraticRegulator lqr;

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

Timer tim;
float delta_t;

// Main program
int main() 
{
    // Initializations
    we.init();
    att_est.init();  
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, 1e6);
    tic_print.attach_us(&callback_print, 1e5);
    tim.start();
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            delta_t = tim.read();
            tim.reset();
            flag = false;
            we.predict(tau);
            we.correct();
            att_est.estimate();
            theta_s = pi/4-att_est.theta_s;
            omega_s = -att_est.omega_s;
            theta_w = -we.theta_w;
            omega_w = -we.omega_w;

            lqr.regulate(theta_s, omega_s, theta_w, omega_w);
            fbl.linearize(lqr.u, theta_s, omega_w);

            if (abs(theta_s) <= 5.0*pi/180.0)
            {
                tau = -fbl.tau;
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
            pc.printf("Theta_s:%6.2f | Omega_s:%6.2f | Theta_w:%6.2f | Omega_w:%6.2f | Torque:%6.3f | Dt:%6.3f\n",theta_s,omega_s,theta_w,omega_w,tau,delta_t);
        }
    }
}