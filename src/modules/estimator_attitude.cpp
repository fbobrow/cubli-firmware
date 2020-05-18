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

    // Normalize rotation quaternion
    float q_norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
    q0 /= q_norm;
    q1 /= q_norm;
    q2 /= q_norm;
    q3 /= q_norm;  
}

// Estimate step
void AttitudeEstimator::predict(float omega_x, float omega_y, float omega_z)
{   
    // Predict rotation quaternion time derivative
    float q0_dot = 0.5*( - q1*omega_x - q2*omega_y - q3*omega_z);
    float q1_dot = 0.5*(   q0*omega_x - q3*omega_y + q2*omega_z);
    float q2_dot = 0.5*(   q3*omega_x + q0*omega_y - q1*omega_z);
    float q3_dot = 0.5*( - q2*omega_x + q0*omega_z + q1*omega_y);
    // Predict rotation quaternion
    q0 += q0_dot*dt;
    q1 += q1_dot*dt;
    q2 += q2_dot*dt;
    q3 += q3_dot*dt;
}

// Correct step
void AttitudeEstimator::correct(float ax, float ay, float az)
{    
    // Calculate quaternion measurement
    float q0_m =   ax*q2 - ay*q1 - az*q0;
    float q1_m = - ax*q3 - ay*q0 + az*q1;
    float q2_m =   ax*q0 - ay*q3 + az*q2;
    float q3_m = - ax*q1 - ay*q2 - az*q3;
    // Correct quaternion time derivative
    q0 += lds*dt*(q0_m-q0);
    q1 += lds*dt*(q1_m-q1);
    q2 += lds*dt*(q2_m-q2);
    q3 += lds*dt*(q3_m-q3);
}