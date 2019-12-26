#include "ahrs.h"

// Class constructor
AHRS::AHRS() : imu(IMU_SDA,IMU_SCL)
{
    // Initialize quaternion and angular velocity vector
    q = eye(4,1);
    omega = zeros(3,1);
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
    Matrix gyr(3,1), acc(3,1), mag(3,1);
    gyr(1,1) = imu.gx;
    gyr(2,1) = imu.gy;
    gyr(3,1) = imu.gz;
    acc(1,1) = f_ax*(imu.ax-b_ax);
    acc(2,1) = f_ay*(imu.ay-b_ay);
    acc(3,1) = f_az*(imu.az-b_az);
    mag(1,1) = f_mx*(imu.mx-b_mx);
    mag(2,1) = f_my*(imu.my-b_my);
    mag(3,1) = f_mz*(imu.mz-b_mz);
    // Extended Kalman filter predict step
    ekf.predict(gyr);
    // Triad update
    triad.update(acc,mag);
    // Extended Kalman filter correction step
    ekf.correct(triad.q);
    // Update orientation quaternion and angular velocity vector
    q(1,1) = ekf.x(1,1);
    q(2,1) = ekf.x(2,1);
    q(3,1) = ekf.x(3,1);
    q(4,1) = ekf.x(4,1);
    omega(1,1) = gyr(1,1)-ekf.x(5,1);
    omega(2,1) = gyr(2,1)-ekf.x(6,1);
    omega(3,1) = gyr(3,1)-ekf.x(7,1);
}
            