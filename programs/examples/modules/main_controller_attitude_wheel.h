#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Motor motor_1(M1_ENABLE,M1_CURRENT), motor_2(M2_ENABLE,M2_CURRENT), motor_3(M3_ENABLE,M3_CURRENT);
WheelEstimator whe_est_1(M1_SPEED), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est(IMU_SDA,IMU_SCL);
AttitudeWheelController cont;
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

// Quaternion error
float phi;
float phi_lim = phi_min;

// Torques
float tau_1, tau_2, tau_3;

float qrt0 = qr0;
float qrt1 = qr1;
float qrt2 = qr2;
float qrt3 = qr3;

float qrt0_dot, qrt1_dot, qrt2_dot, qrt3_dot;

float omega_r_max = 0.5;
float omega_r = 0.0;
float omega_r_x, omega_r_y, omega_r_z;

Timer tim;

// Main program
int main() 
{
    tim.start();
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
            cont.control(qrt0,qrt1,qrt2,qrt3,att_est.q0,att_est.q1,att_est.q2,att_est.q3,omega_r_x,omega_r_y,omega_r_z,att_est.omega_x,att_est.omega_y,att_est.omega_z,whe_est_1.theta_w,whe_est_2.theta_w,whe_est_3.theta_w,whe_est_1.omega_w,whe_est_2.omega_w,whe_est_3.omega_w); 
            phi = 2.0*acos(cont.qe0);
            if (abs(phi) <= phi_lim)
            {
                phi_lim = phi_max;
                tau_1 = cont.tau_1;
                tau_2 = cont.tau_2;
                tau_3 = cont.tau_3;

                if (tim.read() < 2.0)
                {
                    omega_r += (omega_r_max/2.0)*dt;
                }
                else if ((tim.read() > 2) &&  (tim.read() <= 12))
                {
                    omega_r = omega_r_max;
                }
                else if ((tim.read() > 12) &&  (tim.read() <= 16))
                {
                    omega_r -= (omega_r_max/2.0)*dt;
                }
                else if ((tim.read() > 16) &&  (tim.read() <= 26))
                {
                    omega_r = -omega_r_max;
                }
                else if ((tim.read() > 26) &&  (tim.read() <= 28))
                {
                    omega_r += (omega_r_max/2.0)*dt;
                }
                else if (tim.read() > 28)
                {
                    tim.reset();
                }
                omega_r_x = omega_r/sqrt(3.0);
                omega_r_y = omega_r/sqrt(3.0);
                omega_r_z = omega_r/sqrt(3.0);
                qrt0_dot = 0.5*(-qrt1*omega_r_x - qrt2*omega_r_y - qrt3*omega_r_z);
                qrt1_dot = 0.5*( qrt0*omega_r_x - qrt3*omega_r_y + qrt2*omega_r_z);
                qrt2_dot = 0.5*( qrt3*omega_r_x + qrt0*omega_r_y - qrt1*omega_r_z);
                qrt3_dot = 0.5*(-qrt2*omega_r_x + qrt0*omega_r_z + qrt1*omega_r_y);
                qrt0 += qrt0_dot*dt;
                qrt1 += qrt1_dot*dt;
                qrt2 += qrt2_dot*dt;
                qrt3 += qrt3_dot*dt;
            }
            else 
            {
                phi_lim = phi_min;
                tau_1 = 0.0;
                tau_2 = 0.0;
                tau_3 = 0.0;
            }
            motor_1.set_torque(tau_1);
            motor_2.set_torque(tau_2);
            motor_3.set_torque(tau_3);
        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_print) 
        {
            flag_print = false;
            pc.printf("%8.2f\n",omega_r);
            //pc.printf("%8.2f\t%8.2f\t%8.2f\t%8.2f\n",phi*180.0/pi,tau_1/Km,tau_2/Km,tau_3/Km);
            //pc.printf("%5.2f %5.2f %5.2f %5.2f | %6.2f %6.2f %6.2f | %10.2f %10.2f %10.2f | %8.2f %8.2f %8.2f\n",cont.qe0,cont.qe1,cont.qe2,cont.qe3,att_est.omega_x,att_est.omega_y,att_est.omega_z,whe_est_1.theta_w,whe_est_2.theta_w,whe_est_3.theta_w,whe_est_1.omega_w,whe_est_2.omega_w,whe_est_3.omega_w); 
            //pc.printf("%8.2f\t%8.2f\t%8.2f\t%8.2f\n",cont.qe0,cont.qe1,cont.qe2,cont.qe3);
            //pc.printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\n",kp,kd,kpw,kdw);
        }
    }
}