#include "ahrs.h"

// Class constructor
AHRS::AHRS() : imu(IMU_SDA,IMU_SCL)
{
    // Initialize quaternion and angular velocity vector
    q = eye(4,1);
    omega = zeros(3,1);
    a = zeros(3,1);
    m = zeros(3,1);
}

// Class initializer
void AHRS::init()
{
    // Initialize IMU object
    imu.init();
}

// Update step
void AHRS::update()
{
    // Read IMU data
    imu.read();
    // Assignf gyroscope, accelerometer and magnetometer data to vectors 
    Matrix g(3,1);//, a(3,1), m(3,1);
    g(1,1) = imu.gx;
    g(2,1) = imu.gy;
    g(3,1) = imu.gz;
    a(1,1) = f_ax*(imu.ax-b_ax);
    a(2,1) = f_ay*(imu.ay-b_ay);
    a(3,1) = f_az*(imu.az-b_az);
    /*m(1,1) = f_mx*(imu.mx-b_mx);
    m(2,1) = f_my*(imu.my-b_my);
    m(3,1) = f_mz*(imu.mz-b_mz);*/

    m(1,1) = f_mx*(imu.mx-b_mx)+f_mxy*(imu.my-b_my)+f_mxz*(imu.mz-b_mz);
    m(2,1) = f_my*(imu.my-b_my)+f_mxy*(imu.mx-b_mx)+f_myz*(imu.mz-b_mz);
    m(3,1) = f_mz*(imu.mz-b_mz)+f_myz*(imu.my-b_my)+f_mxz*(imu.mx-b_mx);
    // Extended Kalman filter predict step
    ekf.predict(g);
    // Triad update
    triad.update(a,m);
    // Extended Kalman filter correction step
    ekf.correct(triad.q);
    // Update orientation quaternion and angular velocity vector
    q(1,1) = ekf.x(1,1);
    q(2,1) = ekf.x(2,1);
    q(3,1) = ekf.x(3,1);
    q(4,1) = ekf.x(4,1);
    omega(1,1) = g(1,1)-ekf.x(5,1);
    omega(2,1) = g(2,1)-ekf.x(6,1);
    omega(3,1) = g(3,1)-ekf.x(7,1);
}
            