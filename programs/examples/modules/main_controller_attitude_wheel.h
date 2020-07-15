#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Motor motor_1(M1_ENABLE,M1_CURRENT), motor_2(M2_ENABLE,M2_CURRENT), motor_3(M3_ENABLE,M3_CURRENT);
WheelEstimator whe_est_1(M1_SPEED), whe_est_2(M2_SPEED), whe_est_3(M3_SPEED);
AttitudeEstimator att_est(IMU_SDA,IMU_SCL);
AttitudeWheelController cont;
AttitudeTrajectory att_tra;
Ticker tic, tic_log, tic_blink, tic_print;

// Interrupt flags and callback functions
bool flag = false;
bool flag_log = false;
bool flag_blink = false;
bool flag_print = false;
void callback() { flag = true; }
void callback_log() { flag_log = true; }
void callback_blink() { flag_blink = true; }
void callback_print() { flag_print = true; }

// Quaternion error
float phi;
float phi_lim = phi_min;

// Trajectory flag
bool flag_tra = true;

// Security flags
bool flag_arm = false;
bool flag_terminate = false;

// Torques
float tau_1, tau_2, tau_3;

// Serial commands
char command;

// Log variables
float log_data[256][16];
int i = 0;

// Flash memmory
FlashIAP flash;
const uint32_t size = sizeof(log_data);
const uint32_t addr = flash.get_flash_start()+flash.get_flash_size()-size;

// Main program
int main() 
{
    // Initializations
    whe_est_1.init();
    whe_est_2.init();
    whe_est_3.init();
    att_est.init();  
    tic.attach_us(&callback, dt_us);
    // tic_log.attach_us(&callback_log, dt_log_us);
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
            cont.control(att_tra.qr0,att_tra.qr1,att_tra.qr2,att_tra.qr3,att_est.q0,att_est.q1,att_est.q2,att_est.q3,att_tra.omega_r_x,att_tra.omega_r_y,att_tra.omega_r_z,att_est.omega_x,att_est.omega_y,att_est.omega_z,att_tra.alpha_r_x,att_tra.alpha_r_y,att_tra.alpha_r_z,whe_est_1.theta_w,whe_est_2.theta_w,whe_est_3.theta_w,whe_est_1.omega_w,whe_est_2.omega_w,whe_est_3.omega_w); 
            phi = 2.0*acos(cont.qe0);
            if ((abs(phi) <= phi_lim) && !flag_terminate)
            {
                flag_arm = true;
                phi_lim = phi_max;
                if (flag_tra)
                {
                    flag_tra = false;
                    att_tra.init();
                }
                att_tra.generate();
                tau_1 = cont.tau_1;
                tau_2 = cont.tau_2;
                tau_3 = cont.tau_3;
                if (flag_log)
                {
                    flag_log = false;
                    log_data[i][0] = att_est.q0;
                    log_data[i][1] = att_est.q1;
                    log_data[i][2] = att_est.q2;
                    log_data[i][3] = att_est.q3;
                    log_data[i][4] = att_est.omega_x;
                    log_data[i][5] = att_est.omega_y;
                    log_data[i][6] = att_est.omega_z;
                    log_data[i][7] = whe_est_1.theta_w;
                    log_data[i][8] = whe_est_2.theta_w;
                    log_data[i][9] = whe_est_3.theta_w;
                    log_data[i][10] = whe_est_1.omega_w;
                    log_data[i][11] = whe_est_2.omega_w;
                    log_data[i][12] = whe_est_3.omega_w;
                    log_data[i][13] = tau_1;
                    log_data[i][14] = tau_2;
                    log_data[i][15] = tau_3;
                    i++;
                    if (i>255)
                    {
                        flag_terminate = true;
                        flash.init();
                        flash.erase(addr, size);
                        flash.program(log_data, addr, size);   
                        flash.deinit(); 
                    }
                }
            }
            else 
            {
                phi_lim = phi_min;
                tau_1 = 0.0;
                tau_2 = 0.0;
                tau_3 = 0.0;
                if(flag_arm)
                {
                    flag_terminate = true;
                    tic_blink.detach();
                    led = true;
                }
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
            // pc.printf("%8.2f\t%8.2f\t%8.2f\t%8.2f\n",phi*180.0/pi,tau_1/Km,tau_2/Km,tau_3/Km);
            // pc.printf("%5.2f %5.2f %5.2f %5.2f | %6.2f %6.2f %6.2f | %10.2f %10.2f %10.2f | %8.2f %8.2f %8.2f\n",cont.qe0,cont.qe1,cont.qe2,cont.qe3,att_est.omega_x,att_est.omega_y,att_est.omega_z,whe_est_1.theta_w,whe_est_2.theta_w,whe_est_3.theta_w,whe_est_1.omega_w,whe_est_2.omega_w,whe_est_3.omega_w); 
            // pc.printf("%8.2f\t%8.2f\t%8.2f\t%8.2f\n",cont.qe0,cont.qe1,cont.qe2,cont.qe3);
            // pc.printf("%8.4f\t%8.4f\t%8.4f\t%8.4f\n",kp,kd,kpw,kdw);
        }
        if (pc.readable()) 
        {
            command = pc.getc();
            if (command == 'p') 
            {
                flash.init();
                flash.read(log_data, addr, size);
                flash.deinit();
                for(int r=0;r<256;r++)
                {
                    for (int c=0;c<16;c++)
                    {
                        pc.printf("%.4f\t",log_data[r][c]);
                    }
                    pc.printf("\n");
                }
                pc.printf("\n");
            }
        }
    }
}