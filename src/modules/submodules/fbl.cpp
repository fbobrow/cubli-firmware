#include "fbl.h"

// Constructor
FeedbackLinearization::FeedbackLinearization()
{
}

// 
void FeedbackLinearization::linearize(float u, float theta_s, float omega_w)
{
    // Friction torque
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
    // Torque
    tau = m_c*g*d*sin(theta_s)+tau_f-(I_c-I_w)*u;
    // Saturation
    if (tau > 0.1)
    {
        tau = 0.1;
    } 
    else if (tau < -0.1)
    {
        tau = -0.1;
    }
}      