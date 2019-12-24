#include "we.h"

// Constructor
WheelEstimator::WheelEstimator(PinName PIN_SPEED) : hall(M1_SPEED)
{
    theta_w = 0.0;
    omega_w = 0.0;
}

// Initializer
void WheelEstimator::init()
{
    calibrate();
}

// Angular velocity bias calibration 
void WheelEstimator::calibrate()
{
    for(int i = 0; i<200;i++)
    {
        hall.read();
        b_omega_w += hall.omega/200.0;
        wait_us(dt_us);
    }
}

// Predict step
void WheelEstimator::predict(float tau)
{
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
    float alpha_w = (1.0/I_w)*(tau-tau_f);

    theta_w = theta_w+omega_w*dt+alpha_w*pow(dt,2)/2.0;
    omega_w = omega_w+alpha_w*dt;
}

// Correct step
void WheelEstimator::correct()
{
    hall.read();
    float omega_wm = hall.omega-b_omega_w;
    omega_w = omega_w+ldw*dt*(omega_wm-omega_w);

}
            