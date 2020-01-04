#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Motor motor(M1_ENABLE,M1_CURRENT);
WheelEstimator whe_est(M1_SPEED);
AttitudeEstimator att_est;
AttitudeWheelController2D cont;
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

//
float theta_s, omega_s, theta_w, omega_w;
float tau;

//
float q0, q1, q2, q3;
float theta_x, theta_y, theta_z;

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
            att_est.estimate();
            // Convert quaternions into Euler angles
            q0 = att_est.q0;
            q1 = att_est.q1;
            q2 = att_est.q2;
            q3 = att_est.q3;
            theta_z = atan2(2.0*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
            theta_y = asin(-2.0*(q1*q3-q0*q2));
            theta_x = atan2(2.0*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
            // Get 2D angles and angular velocities
            theta_s = 44.5*pi/180.0-theta_x;
            omega_s = -att_est.omega_x;
            theta_w = -whe_est.theta_w;
            omega_w = -whe_est.omega_w;
            /*theta_s = 41.7*pi/180.0+theta_y;
            omega_s = att_est.omega_y;
            theta_w = whe_est.theta_w;
            omega_w = whe_est.omega_w;*/
            /*theta_s = 45.0*pi/180.0-theta_z;
            omega_s = -att_est.omega_z;
            theta_w = -whe_est.theta_w;
            omega_w = -whe_est.omega_w;*/
            // Control torque
            cont.control(theta_s, omega_s, theta_w, omega_w);
            if (abs(theta_s) <= 5.0*pi/180.0)
            {
                tau = -cont.tau;
                //tau = cont.tau;
                //tau = -cont.tau;
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