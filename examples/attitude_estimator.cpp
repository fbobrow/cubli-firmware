#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
LSM9DS1 imu(D4,D5);
Estimator estimator(250);

// MATLAB comand
char command;

// Ticker
Ticker tic_blink;
Ticker tic_imu;
Ticker tic_estimator;

// Flags
bool flag_blink = false;
bool flag_imu = false;
bool flag_estimator = false;


// Callbacks
void callback_blink()
{
    flag_blink = true;
}
void callback_imu()
{
    flag_imu = true;
}
void callback_estimator()
{
    flag_estimator = true;
}

Timer tim;
float dt;

// Main program
int main()
{
    imu.init();  
    tic_blink.attach(&callback_blink, 0.5);
    tic_imu.attach(&callback_imu, 1.0/250);
    tic_estimator.attach(&callback_estimator, 1.0/250);
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
        }
        if (flag_estimator) 
        {
            flag_estimator = false;
            tim.reset();
            imu.read();
            estimator.update(imu.gx,imu.gy,imu.gz,imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz);
            dt = tim.read();
        }
        if (pc.readable()) {
            command = pc.getc();
            if (command == 'p') {
                pc.printf("%f,%f,%f,%f\n",estimator.q(1,1),estimator.q(2,1),estimator.q(3,1),estimator.q(4,1));
                //pc.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f\n",imu.gx,imu.gy,imu.gz,imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz);
                //pc.printf("%.2fms\n",dt*1000.0f);
            }
        }
        //pc.printf("%6.2f,%6.2f,%6.2f | %6.2f,%6.2f,%6.2f\n",imu.ax,imu.ay,imu.az,imu.mx,imu.my,imu.mz);
    }
}