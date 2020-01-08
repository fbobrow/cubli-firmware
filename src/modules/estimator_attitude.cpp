#include "estimator_attitude.h"

// Constructor
AttitudeEstimator::AttitudeEstimator(PinName PIN_SDA, PinName PIN_SCL) : imu(PIN_SDA,PIN_SCL)
{
    // Set initial quaternion
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
    // Calculate angular velocity bias by averaging f samples during 1 second
    int n = f;
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
    // Read IMU data
    imu.read();
    // Get angular velocity from gyroscope readings
    omega_x = imu.gx-b_omega_x;
    omega_y = imu.gy-b_omega_y;
    omega_z = imu.gz-b_omega_z;
    // Get linear accelration from gyroscope readings
    float ax = f_ax*(imu.ax-b_ax);
    float ay = f_ay*(imu.ay-b_ay);
    float az = f_az*(imu.az-b_az);
    // Normalize accelerometer measurement
    float a_norm = sqrt(ax*ax+ay*ay+az*az);
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;   
	// Calculate quaternion time derivative
	float q0_dot = 0.5*(-q1*omega_x - q2*omega_y - q3*omega_z);
	float q1_dot = 0.5*( q0*omega_x - q3*omega_y + q2*omega_z);
	float q3_dot = 0.5*( q3*omega_x + q0*omega_y - q1*omega_z);
	float q4_dot = 0.5*(-q2*omega_x + q0*omega_z + q1*omega_y);
    // Gradient decent algorithm corrective step (with auxiliary variables to avoid double arithmetic)
    float _2q0 = 2.0*q0;
    float _2q1 = 2.0*q1;
    float _2q2 = 2.0*q2;
    float _2q3 = 2.0*q3;
    float s0 = (1.0 + az)*_2q0 - ax*_2q2 + ay*_2q1;
    float s1 = (1.0 - az)*_2q1 + ax*_2q3 + ay*_2q0;
    float s2 = (1.0 - az)*_2q2 - ax*_2q0 + ay*_2q3;
    float s3 = (1.0 + az)*_2q3 + ax*_2q1 + ay*_2q2;
    // Normalise step magnitude
    float s_norm = sqrt(s0*s0+s1*s1+s2*s2+s3*s3); 
    s0 /= s_norm;
    s1 /= s_norm;
    s2 /= s_norm;
    s3 /= s_norm;
    // Apply feedback step
    q0_dot -= lds*dt*s0;
    q1_dot -= lds*dt*s1;
    q3_dot -= lds*dt*s2;
    q4_dot -= lds*dt*s3;
	// Integrate quaternion time derivative
	q0 += q0_dot*dt;
	q1 += q1_dot*dt;
	q2 += q3_dot*dt;
	q3 += q4_dot*dt;
	// Normalize quaternion
	float q_norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0 /= q_norm;
	q1 /= q_norm;
	q2 /= q_norm;
	q3 /= q_norm;   
}