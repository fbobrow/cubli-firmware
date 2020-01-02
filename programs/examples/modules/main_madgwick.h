#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Madgwick att_est(IMU_SDA,IMU_SCL);
Ticker tic, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_blink = false;
void callback() { flag = true; }
void callback_blink() { flag_blink = true; }

// Serial commands
char command;

Timer tim;
float delta_t;

float q0, q1, q2, q3;
float theta_x, theta_y, theta_z;

// Main program
int main() 
{
    // Initializations
    tim.start();
    att_est.init();
    tic.attach_us(&callback, dt_us);
    tic_blink.attach_us(&callback_blink, dt_blink_us);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
            delta_t = tim.read();
            tim.reset();
            att_est.estimate();
            q0 = att_est.q0;
            q1 = att_est.q1;
            q2 = att_est.q2;
            q3 = att_est.q3;
            theta_z = atan2(2.0*(q1*q2+q0*q3),q0*q0+q1*q1-q2*q2-q3*q3);
            theta_y = asin(-2.0*(q1*q3-q0*q2));
            theta_x = atan2(2.0*(q2*q3+q0*q1),q0*q0-q1*q1-q2*q2+q3*q3);
        }
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%.4f,%.4f,%.4f,%.4f\n",att_est.q0,att_est.q1,att_est.q2,att_est.q3);
                pc.printf("%.4f,%.4f,%.4f\n",theta_x,theta_y,theta_z);
                pc.printf("%.4f\n",delta_t);
            }
        }
    }
}