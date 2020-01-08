#include "estimator_attitude.h"

// Constructor
AttitudeEstimator::AttitudeEstimator(PinName PIN_SDA, PinName PIN_SCL) : imu(PIN_SDA,PIN_SCL)
{
    // Set initial rotation quaternion
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    // Set initial angular velocity
    omega_x = 0.0;
    omega_y = 0.0;
    omega_z = 0.0;
    // Set initial rotation quaternion time derivative
    q0_dot = 0.0;
    q1_dot = 0.0;
    q2_dot = 0.0;
    q3_dot = 0.0;
    // Set initial angular velocity bias
    b_omega_x = 0.0;
    b_omega_y = 0.0;
    b_omega_z = 0.0;
}

// Initializer
void AttitudeEstimator::init()
{
    // Initialize IMU sensor object
    imu.init();
    // Angular velocity bias calibration
    calibrate();
}

// Angular velocity bias calibration 
void AttitudeEstimator::calibrate()
{
    // Calculate angular velocity bias by averaging n samples during 0,5 seconds
    int n = f/2;
    for(int i = 0; i<f;i++)
    {
        imu.read();
        b_omega_x += imu.gx/f;
        b_omega_y += imu.gy/f;
        b_omega_z += imu.gz/f;
        wait_us(dt_us);
    }
}

// Estimate step
void AttitudeEstimator::estimate()
{   
    // Get angular velocity from IMU gyroscope data
    imu.read_gyr();
    omega_x = imu.gx-b_omega_x;
    omega_y = imu.gy-b_omega_y;
    omega_z = imu.gz-b_omega_z;
    // Predict step
    predict(omega_x,omega_y,omega_z);

    // Get linear acceleration from IMU accelerometer data
    imu.read_acc();
    float ax = f_ax*(imu.ax-b_ax);
    float ay = f_ay*(imu.ay-b_ay);
    float az = f_az*(imu.az-b_az);
    // Normalize linear acceleration
    float a_norm = sqrt(ax*ax+ay*ay+az*az);
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;  
    // Correct step
    correct(ax,ay,az);
}

// Estimate step
void AttitudeEstimator::predict(float omega_x, float omega_y, float omega_z)
{   
    // Predict rotation quaternion
    q0 += q0_dot*dt;
    q1 += q1_dot*dt;
    q2 += q2_dot*dt;
    q3 += q3_dot*dt;
    // Normalize rotation quaternion
    float q_norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    q0 /= q_norm;
    q1 /= q_norm;
    q2 /= q_norm;
    q3 /= q_norm;  
    // Predict rotation quaternion time derivative
    q0_dot = 0.5*(-q1*omega_x - q2*omega_y - q3*omega_z);
    q1_dot = 0.5*( q0*omega_x - q3*omega_y + q2*omega_z);
    q2_dot = 0.5*( q3*omega_x + q0*omega_y - q1*omega_z);
    q3_dot = 0.5*(-q2*omega_x + q0*omega_z + q1*omega_y);
}

// Correct step
void AttitudeEstimator::correct(float ax, float ay, float az)
{    
    // Gradient decent algorithm corrective step
    float q0_dot_e = -2.0*((1.0 + az)*q0 - ax*q2 + ay*q1);
    float q1_dot_e = -2.0*((1.0 - az)*q1 + ax*q3 + ay*q0);
    float q2_dot_e = -2.0*((1.0 - az)*q2 - ax*q0 + ay*q3);
    float q3_dot_e = -2.0*((1.0 + az)*q3 + ax*q1 + ay*q2);
    // Normalise corrective step
    float q_dot_e_norm = sqrt(q0_dot_e*q0_dot_e+q1_dot_e*q1_dot_e+q2_dot_e*q2_dot_e+q3_dot_e*q3_dot_e); 
    q0_dot_e /= q_dot_e_norm;
    q1_dot_e /= q_dot_e_norm;
    q2_dot_e /= q_dot_e_norm;
    q3_dot_e /= q_dot_e_norm;
    // Correct quaternion time derivative
    q0_dot += lds*dt*q0_dot_e;
    q1_dot += lds*dt*q1_dot_e;
    q2_dot += lds*dt*q2_dot_e;
    q3_dot += lds*dt*q3_dot_e; 
}