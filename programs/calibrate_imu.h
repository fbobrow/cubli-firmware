#include "mbed.h"
#include "src/config/parameters.h"
#include "src/drivers/lsm9ds1.h"
#include "src/utils/matrix.h"

// Objects
Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
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

Matrix g_list(n,3), a_list(n,3), m_list(n,3);
Matrix g(3,1), a(3,1), m(3,1);

// Main program
int main()
{
    imu.init();  
    tic.attach_us(&callback, 10*dt_us);
    while (true) 
    {
        if (flag)
        {
            flag = false;
            imu.read();
            g(1,1) = imu.gx;
            g(2,1) = imu.gy;
            g(3,1) = imu.gz;
            a(1,1) = f_ax*(imu.ax-b_ax);
            a(2,1) = f_ay*(imu.ay-b_ay);
            a(3,1) = f_az*(imu.az-b_az);
            m(1,1) = f_mx*(imu.mx-b_mx)+f_mxy*(imu.my-b_my)+f_mxz*(imu.mz-b_mz);
            m(2,1) = f_my*(imu.my-b_my)+f_mxy*(imu.mx-b_mx)+f_myz*(imu.mz-b_mz);
            m(3,1) = f_mz*(imu.mz-b_mz)+f_myz*(imu.my-b_my)+f_mxz*(imu.mx-b_mx);
            g_list(k,1) = imu.gx;
            g_list(k,2) = imu.gy;
            g_list(k,3) = imu.gz;
            a_list(k,1) = imu.ax;
            a_list(k,2) = imu.ay;
            a_list(k,3) = imu.az;
            m_list(k,1) = imu.mx;
            m_list(k,2) = imu.my;
            m_list(k,3) = imu.mz;
            k++;
            //pc.printf("%6.2f %6.2f %6.2f | %6.2f\n",g(1,1),g(2,1),g(3,1),norm(g));
            //pc.printf("%6.2f %6.2f %6.2f | %6.2f\n",a(1,1),a(2,1),a(3,1),norm(a));
            //pc.printf("%6.2f %6.2f %6.2f | %6.2f\n",m(1,1),m(2,1),m(3,1),norm(m));
            pc.printf("%6.2f\t%6.2f\t%6.2f\n",m(1,1),m(2,1),m(3,1));
            //pc.printf("%6.2f\t%6.2f\t%6.2f\n",imu.mx,imu.my,imu.mz);
        }
        if (k > n) 
        {
            k = 1;
            /*for (int i = 1; i <= n; i++) {
                for (int j = 1; j <= 3; j++) {
                    pc.printf("%.8f",m_list(i,j));
                    if (j < 3) {
                        pc.printf("\t");
                    }
                }
                pc.printf("\n");
            }
            pc.printf("\n");*/
        }
    }
}