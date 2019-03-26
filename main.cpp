#include "mbed.h"
#include "LSM9DS1.h"

DigitalOut led(LED1);
LSM9DS1 imu(PB_7,PB_6);
Serial pc(USBTX,USBRX,NULL,230400);

Thread sensors;

void sensors_thread()
{
    while(true)
    {
        imu.read();
        wait(0.5);
    }
}

int main() {
    sensors.start(callback(sensors_thread));
    imu.init();
    while(true) {
        led = !led;
        pc.printf("Acc [m/s^2]: %4.2f | %4.2f | %4.2f \n", imu.ax, imu.ay, imu.az);
        pc.printf("Gyr [rad/s]: %4.2f | %4.2f | %4.2f \n", imu.gx, imu.gy, imu.gz);
        pc.printf("Mag    [uT]: %4.2f | %4.2f | %4.2f \n\n", imu.mx, imu.my, imu.mz);
        wait(0.1);
    }
}
