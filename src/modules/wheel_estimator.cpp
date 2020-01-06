#include "wheel_estimator.h"

// Constructor
WheelEstimator::WheelEstimator(PinName PIN_SPEED) : hall(PIN_SPEED)
{
    theta_w = 0.0;
    omega_w = 0.0;
}

// Initializer
void WheelEstimator::init()
{
    calibrate();
}

// Estimate step
void WheelEstimator::estimate(float tau)
{
    predict(tau);
    correct();
}

// Angular velocity bias calibration 
void WheelEstimator::calibrate()
{
    // Calculate angular velocity bias by averaging f samples durint 1 second
    int n = f;
    for(int i = 0; i<n;i++)
    {
        hall.read();
        b_omega_w += hall.omega/n;
        wait_us(dt_us);
    }
}

// Predict step
void WheelEstimator::predict(float tau)
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
    float omega_w_dot = (1.0/I_w)*(-tau_f+tau);
    // Predict angular displacement and angular velocity
    theta_w = theta_w+omega_w*dt+omega_w_dot*dt*dt/2.0;
    omega_w = omega_w+omega_w_dot*dt;
}

// Correct step
void WheelEstimator::correct()
{
    // Get angular velocity measurement
    hall.read();
    float omega_w_m = hall.omega-b_omega_w;
    // Correct angular velocity with measurement
    omega_w = omega_w+ldw*dt*(omega_w_m-omega_w);
}
            