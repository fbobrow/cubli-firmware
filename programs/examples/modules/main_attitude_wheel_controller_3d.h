#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Motor motor_1(M1_ENABLE,M1_CURRENT), motor_2(M2_ENABLE,M2_CURRENT), motor_3(M3_ENABLE,M3_CURRENT);
WheelEstimator whe_est_1(M1_SPEED), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est;
AttitudeWheelController3D cont;
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

//
float phi, sign;
float tau_1, tau_2, tau_3;

// Quaternion reference

//Vertex
/*float q0r = 0.8881;
float q1r = 0.3251;
float q2r = -0.3251;
float q3r = 0.0000;*/

// Edge (x axis)
float q0r = 0.9239;
float q1r = 0.3827;
float q2r = 0.0000;
float q3r = 0.0000;

// Edge (y axis)
/*float q0r = 0.9239;
float q1r = 0.0000;
float q2r = 0.3827;
float q3r = 0.0000;*/

// Main program
int main() 
{
    // Initializations
    whe_est_1.init();
    whe_est_2.init();
    whe_est_3.init();
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
            whe_est_1.estimate(tau_1);
            whe_est_2.estimate(tau_2);
            whe_est_3.estimate(tau_3);
            att_est.estimate();
            cont.control(q0r,q1r,q2r,q3r,att_est.q0,att_est.q1,att_est.q2,att_est.q3,att_est.omega_x,att_est.omega_y,att_est.omega_z,whe_est_1.theta_w,whe_est_2.theta_w,whe_est_3.theta_w,whe_est_1.omega_w,whe_est_2.omega_w,whe_est_3.omega_w);
            phi = 2.0*acos(q0r*att_est.q0 + q1r*att_est.q1 + q2r*att_est.q2 + q3r*att_est.q3);
            sign = q1r*att_est.q0 - q0r*att_est.q1 - q3r*att_est.q2 + q2r*att_est.q3;
            if (sign < 0)
            {
                phi = -phi;
            }
            if (abs(phi) <= 5.0*pi/180.0)
            {
                tau_1 = cont.tau_1;
                tau_2 = cont.tau_2;
                tau_3 = cont.tau_3;
            }
            else 
            {
                tau_1 = 0.0;
                tau_2 = 0.0;
                tau_3 = 0.0;
            }
            //motor_1.set_torque(tau_1);
            //motor_2.set_torque(tau_2);
            //motor_3.set_torque(tau_3);
        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_print) 
        {
            flag_print = false;
            pc.printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\n",phi,tau_1/Km,tau_2/Km,tau_3/Km);
            //pc.printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\n",att_est.q0,att_est.q1,att_est.q2,att_est.q3);
        }
    }
}