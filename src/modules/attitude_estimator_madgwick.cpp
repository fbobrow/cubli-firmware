#include "attitude_estimator_madgwick.h"

// Constructor
AttitudeEstimatorMadgwick::AttitudeEstimatorMadgwick(PinName PIN_SDA, PinName PIN_SCL) : imu(PIN_SDA,PIN_SCL)
{
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    omega_x = 0.0;
    omega_y = 0.0;
    omega_z = 0.0;
    b_omega_x = 0.0;
    b_omega_y = 0.0;
    b_omega_z = 0.0;
}

// Initializer
void AttitudeEstimatorMadgwick::init()
{
    imu.init();
    calibrate();
}

// Angular velocity bias calibration 
void AttitudeEstimatorMadgwick::calibrate()
{
    // Calculate angular velocity bias by averaging 200 samples durint 1 second
    for(int i = 0; i<200;i++)
    {
        imu.read();
        b_omega_x += imu.gx/200.0;
        b_omega_y += imu.gy/200.0;
        b_omega_z += imu.gz/200.0;
        wait_us(dt_us);
    }
}

// Estimate step
void AttitudeEstimatorMadgwick::estimate()
{   
    //
    imu.read();

    // Angular velocity from gyroscope readings
    omega_x = imu.gx-b_omega_x;
    omega_y = imu.gy-b_omega_y;
    omega_z = imu.gz-b_omega_z;

	// Rate of change of quaternion
	float q0_dot = 0.5*(-q1*omega_x-q2*omega_y-q3*omega_z);
	float q1_dot = 0.5*(q0*omega_x-q3*omega_y+q2*omega_z);
	float q3_dot = 0.5*(q3*omega_x+q0*omega_y-q1*omega_z);
	float q4_dot = 0.5*(-q2*omega_x+q0*omega_z+q1*omega_y);

    // Accelerometer
    float ax = f_ax*(imu.ax-b_ax);
    float ay = f_ay*(imu.ay-b_ay);
    float az = f_az*(imu.az-b_az);

    // Normalise accelerometer measurement
    float a_norm = -sqrt(ax*ax+ay*ay+az*az);
    ax /= a_norm;
    ay /= a_norm;
    az /= a_norm;   

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0*q0;
    float _2q1 = 2.0*q1;
    float _2q2 = 2.0*q2;
    float _2q3 = 2.0*q3;
    float _4q0 = 4.0*q0;
    float _4q1 = 4.0*q1;
    float _4q2 = 4.0*q2;
    float _8q1 = 8.0*q1;
    float _8q2 = 8.0*q2;
    float q0q0 = q0*q0;
    float q1q1 = q1*q1;
    float q2q2 = q2*q2;
    float q3q3 = q3*q3;

    // Gradient decent algorithm corrective step
    float s0 = _4q0*q2q2+_2q2*ax+_4q0*q1q1-_2q1*ay;
    float s1 = _4q1*q3q3-_2q3*ax+4.0*q0q0*q1-_2q0*ay-_4q1+_8q1*q1q1+_8q1*q2q2+_4q1*az;
    float s2 = 4.0*q0q0*q2+_2q0*ax+_4q2*q3q3-_2q3*ay-_4q2+_8q2*q1q1+_8q2*q2q2+_4q2*az;
    float s3 = 4.0*q1q1*q3-_2q1*ax+4.0*q2q2*q3-_2q2*ay;
    float s_norm = sqrt(s0*s0+s1*s1+s2*s2+s3*s3); // normalise step magnitude
    s0 /= s_norm;
    s1 /= s_norm;
    s2 /= s_norm;
    s3 /= s_norm;

    // Apply feedback step
    float beta = 0.01;
    q0_dot -= beta*s0;
    q1_dot -= beta*s1;
    q3_dot -= beta*s2;
    q4_dot -= beta*s3;

	// Integrate rate of change of quaternion to yield quaternion
	q0 += q0_dot*dt;
	q1 += q1_dot*dt;
	q2 += q3_dot*dt;
	q3 += q4_dot*dt;

	// Normalise quaternion
	float q_norm = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	q0 /= q_norm;
	q1 /= q_norm;
	q2 /= q_norm;
	q3 /= q_norm;
    
}