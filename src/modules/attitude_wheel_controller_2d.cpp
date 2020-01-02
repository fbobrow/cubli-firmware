#include "attitude_wheel_controller_2d.h"

// Constructor
AttitudeWheelController2D::AttitudeWheelController2D()
{
    tau = 0.0;
    u = 0.0;
}

// Control step
void AttitudeWheelController2D::control(float theta_s, float omega_s, float theta_w, float omega_w)
{
    state_regulator(theta_s,omega_s,theta_w,omega_w);
    feedback_linearization(theta_s,omega_w);
}   

// State regulator step
void AttitudeWheelController2D::state_regulator(float theta_s, float omega_s, float theta_w, float omega_w)
{
    u = kps*(0.0-theta_s)+kds*(0.0-omega_s)+kpw*(0.0-theta_w)+kdw*(0.0-omega_w);
}   

// Feedback linearization step
void AttitudeWheelController2D::feedback_linearization(float theta_s, float omega_w)
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
    // Calculate torque
    tau = m_c*g*d*sin(theta_s)+tau_f-(I_c-I_w)*u;
    // Saturation for safety
    if (tau > 0.1)
    {
        tau = 0.1;
    } 
    else if (tau < -0.1)
    {
        tau = -0.1;
    }
}         