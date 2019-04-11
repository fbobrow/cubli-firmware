#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
LSM9DS1 imu(D4,D5);
AttitudeEstimator att_est(250);

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

// Main program
int main()
{
    imu.init();  
    tic_blink.attach(&callback_blink, 0.5);
    tic_imu.attach(&callback_imu, 1.0/250);
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
            tim.reset();
            flag_imu = false;
            imu.read();
            flag_att_est = true;
        }
        if (flag_att_est) 
        {
            flag_att_est = false;
            att_est.update(imu.gx,imu.gy,imu.gz,imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz);
            dt = tim.read();
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                //pc.printf("%f,%f,%f,%f\n",att_est.q(1,1),att_est.q(2,1),att_est.q(3,1),att_est.q(4,1));
                //pc.printf("%f,%f,%f,%f\n",att_est.omega(1,1),att_est.omega(2,1),att_est.omega(3,1));
                pc.printf("%.2fms\n",dt*1000.0f);
            }
        }
    }
}