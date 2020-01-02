#include "attitude_estimator_2d.h"

// Constructor
AttitudeEstimator2D::AttitudeEstimator2D(PinName PIN_SDA, PinName PIN_SCL) : imu(PIN_SDA,PIN_SCL)
{
    theta_s = 0.0;
    omega_s = 0.0;
}

// Initializer
void AttitudeEstimator2D::init()
{
    imu.init();
    calibrate();
}

// Angular velocity bias calibration 
void AttitudeEstimator2D::calibrate()
{
    // Calculate angular velocity bias by averaging 200 samples durint 1 second
    for(int i = 0; i<200;i++)
    {
        imu.read();
        b_omega_s += imu.gx/200.0;
        wait_us(dt_us);
    }
}

// Estimate step
void AttitudeEstimator2D::estimate(float tau, float omega_w)
{
    predict(tau,omega_w);
    correct();
}

// Predict step
void AttitudeEstimator2D::predict(float tau, float omega_w)
{
    // Calculate friction torque
    float sign;
    if (omega_w == 0.0)
    {
        sign = 0.0;
    }
    else
    {
        sign = omega_w/abs(omega_w);
    }
    float tau_f = sign*(tau_c+b*abs(omega_w)+kd*pow(omega_w,2));
    // Calculate angular acceleration
    float omega_s_dot = (1.0/(I_c-I_w))*(tau_f/*+m_c*g*sin(theta_s)*/-tau);
    // Predict angular displacement and angular velocity
    theta_s = theta_s+omega_s*dt+omega_s_dot*dt2_2;
    omega_s = omega_s+omega_s_dot*dt;
}

// Correct step
void AttitudeEstimator2D::correct()
{
    // Get angular velocity measurement
    imu.read();
    float theta_s_m = atan2(-imu.ay,-imu.az);
    float omega_s_m = imu.gx-b_omega_s;
    // Correct angular velocity with measurement
    theta_s = theta_s+lps*dt*(theta_s_m-theta_s);
    omega_s = omega_s+lds*dt*(omega_s_m-omega_s);
}
            