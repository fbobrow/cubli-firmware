#include "we.h"

// Class constructor
WheelEstimator::WheelEstimator(PinName PIN_SPEED) : hall(M1_SPEED)
{
    theta = 0.0;
    omega = 0.0;
}

void WheelEstimator::init()
{
    calibrate();
}

void WheelEstimator::calibrate()
{
    // Calculate angular velocities bias (rad/s) by averaging 200 samples during 1 seconds
    for(int i = 0; i<200;i++)
    {
        hall.read();
        omega_bias += hall.omega/200.0;
        wait_us(dt_us);
    }
}

// Predict step
void WheelEstimator::predict()
{
    theta = theta+omega*dt;
    omega = omega;
}

// Update step
void WheelEstimator::correct()
{
    hall.read();
    /*float*/ omega_m = hall.omega-omega_bias;
    theta = theta+lpw*dt*(omega_m-omega);
    omega = omega+ldw*dt*(omega_m-omega);
}
            