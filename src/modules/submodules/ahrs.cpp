#include "ahrs.h"

// Class constructor
AttitudeHeadingReferenceSystem::AttitudeHeadingReferenceSystem(float freq) : ekf(freq,g_cov,q_cov)
{
    // Initialize quaternion and angular velocity vector
    q = eye(4,1);
    omega = zeros(3,1);
}

// Update step
void AttitudeHeadingReferenceSystem::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    // Gyroscope, accelerometer and magnetometer vectors 
    Matrix g(3,1), a(3,1), m(3,1);
    g(1,1) = gx;
    g(2,1) = gy;
    g(3,1) = gz;
    a(1,1) = f_ax*(ax-b_ax);
    a(2,1) = f_ay*(ay-b_ay);
    a(3,1) = f_az*(az-b_az);
    m(1,1) = f_mx*(mx-b_mx);
    m(2,1) = f_my*(my-b_my);
    m(3,1) = f_mz*(mz-b_mz);
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
            