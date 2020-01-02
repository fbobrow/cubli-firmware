#include "attitude_estimator2.h"

// Constructor
AttitudeEstimator2::AttitudeEstimator2(PinName PIN_SDA, PinName PIN_SCL) : imu(PIN_SDA,PIN_SCL)
{
    // Initialize quaternion and angular velocity vector
    q = eye(4,1);
    omega = zeros(3,1);
}

// Class initializer
void AttitudeEstimator2::init()
{
    // Initialize IMU object
    imu.init();
}

// Update step
void AttitudeEstimator2::estimate()
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
    // Extended Kalman filter predict step
    ekf.predict(gyr);
    // Extended Kalman filter correction step
    ekf.correct(acc);
    // Update orientation quaternion and angular velocity vector
    q(1,1) = ekf.x(1,1);
    q(2,1) = ekf.x(2,1);
    q(3,1) = ekf.x(3,1);
    q(4,1) = ekf.x(4,1);
    omega(1,1) = gyr(1,1)-ekf.x(5,1);
    omega(2,1) = gyr(2,1)-ekf.x(6,1);
    omega(3,1) = gyr(3,1)-ekf.x(7,1);
    P = ekf.P;
}
            