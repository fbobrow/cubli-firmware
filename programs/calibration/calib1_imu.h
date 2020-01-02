#include "mbed.h"
#include "src/config/parameters.h"
#include "src/drivers/lsm9ds1.h"
#include "src/utils/matrix.h"

// Objects
Serial pc(SERIAL_TX, SERIAL_RX,NULL,115200);
LSM9DS1 imu(D4,D5);

// Ticker
Ticker tic;

// Flag
bool flag = false;

// Callback
void callback()
{
    flag = true;
}

int k = 1;
int n = 100;

Matrix gyr_list(n,3), acc_list(n,3), mag_list(n,3);
Matrix gyr(3,1), acc(3,1), mag(3,1);

// Main program
int main()
{
    imu.init();  
    tic.attach_us(&callback, dt_us);
    while (true) 
    {
        if (flag)
        {
            flag = false;
            imu.read();
            gyr(1,1) = imu.gx;
            gyr(2,1) = imu.gy;
            gyr(3,1) = imu.gz;
            acc(1,1) = f_ax*(imu.ax-b_ax);
            acc(2,1) = f_ay*(imu.ay-b_ay);
            acc(3,1) = f_az*(imu.az-b_az);
            mag(1,1) = f_mx*(imu.mx-b_mx)+f_mxy*(imu.my-b_my)+f_mxz*(imu.mz-b_mz);
            mag(2,1) = f_my*(imu.my-b_my)+f_mxy*(imu.mx-b_mx)+f_myz*(imu.mz-b_mz);
            mag(3,1) = f_mz*(imu.mz-b_mz)+f_myz*(imu.my-b_my)+f_mxz*(imu.mx-b_mx);
            gyr_list(k,1) = imu.gx;
            gyr_list(k,2) = imu.gy;
            gyr_list(k,3) = imu.gz;
            acc_list(k,1) = imu.ax;
            acc_list(k,2) = imu.ay;
            acc_list(k,3) = imu.az;
            mag_list(k,1) = imu.mx;
            mag_list(k,2) = imu.my;
            mag_list(k,3) = imu.mz;
            k++;
            //pc.printf("%6.2f %6.2f %6.2f | %6.2f\n",gyr(1,1),gyr(2,1),gyr(3,1),norm(gyr));
            //pc.printf("%6.2f %6.2f %6.2f | %6.2f\n",acc(1,1),acc(2,1),acc(3,1),norm(acc));
            //pc.printf("%6.2f %6.2f %6.2f | %6.2f\n",mag(1,1),mag(2,1),mag(3,1),norm(mag));
            //pc.printf("%6.2f\t%6.2f\t%6.2f\n",mag(1,1),mag(2,1),mag(3,1));
            //pc.printf("%6.2f\t%6.2f\t%6.2f\n",imu.mx,imu.my,imu.mz);
        }
        if (k > n) 
        {
            k = 1;
            for (int i = 1; i <= n; i++) {
                for (int j = 1; j <= 3; j++) {
                    pc.printf("%.8f",gyr_list(i,j));
                    if (j < 3) {
                        pc.printf("\t");
                    }
                }
                pc.printf("\n");
            }
            pc.printf("\n");
        }
    }
}