#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
LSM9DS1 imu(IMU_SDA,IMU_SCL);
Escon motor_1(M1_ENABLE,M1_SPEED,M1_CURRENT);
Escon motor_2(M2_ENABLE,M2_SPEED,M2_CURRENT);
Escon motor_3(M3_ENABLE,M3_SPEED,M3_CURRENT);
AttitudeEstimator att_est(freq_estimator);
SpeedEstimator spe_est(freq_estimator);

// MATLAB comand
char command;

// Ticker
Ticker tic_blink;
Ticker tic_imu;

// Flags
bool flag_blink = false;
bool flag_imu = false;
bool flag_att_est = false;

// Callbacks
void callback_blink()
{
    flag_blink = true;
}
void callback_imu()
{
    flag_imu = true;
}

// Debug timer
Timer tim;
float dt;


float qr_0 = 0.917117;
float qr_1 = -0.038449;
float qr_2 = -0.373732;
float qr_3 = -0.133200;
float Ixx = 0.01f;
float zeta = 0.6f;
float T_s = 0.3f;
float omega_n = 4.0f/(T_s*zeta);
float kp = omega_n*omega_n;
float kd = 2.0f*zeta*omega_n;
float kd_2 = 0.05f;
float tau;

float theta_e, omega_e, omega_w_e;

// Main program
int main()
{
    imu.init();  
    tic_blink.attach(&callback_blink, 1.0/freq_blink);
    tic_imu.attach(&callback_imu, 1.0/freq_estimator);
    tim.start();
    while (true) 
    {
        if (flag_blink)
        {
            flag_blink = false;
            led = !led;
        }
        if (flag_imu)
        {
            flag_imu = false;
            imu.read();
            flag_att_est = true;
        }
        if (flag_att_est) 
        {
            flag_att_est = false;
            att_est.update(imu.gx,imu.gy,imu.gz,imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz);

            motor_1.read();
            motor_2.read();
            //motor_3.read();
            spe_est.update(motor_1.omega,motor_2.omega,motor_3.omega);
            theta_e = 2.0f*(qr_1*att_est.q(1,1)-qr_0*att_est.q(2,1)+qr_3*att_est.q(3,1)-qr_2*att_est.q(4,1));
            omega_e = -att_est.omega(1,1);
            omega_w_e = spe_est.omega(2,1);
            tau = -Ixx*(kp*theta_e+kd*omega_e+kd_2*omega_w_e);
            //pc.printf("t_e: %6.2frad | o_e: %6.2frad/s | o_w_e: %6.2frad/s | tau: %6.2fNm\n",theta_e,omega_e,omega_w_e,tau);
            //motor_2.set_torque(tau);
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%f,%f,%f,%f\n",att_est.q(1,1),att_est.q(2,1),att_est.q(3,1),att_est.q(4,1));
                //pc.printf("%f,%f,%f,%f\n",att_est.omega(1,1),att_est.omega(2,1),att_est.omega(3,1));
            }
        }
    }
}