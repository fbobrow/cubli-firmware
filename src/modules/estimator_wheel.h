#ifndef estimator_wheel_h
#define estimator_wheel_h

#include "mbed.h"

#include "src/utils/parameters.h"
#include "src/utils/pin_names.h"
#include "src/drivers/hall.h"

// Wheel estimator class
class WheelEstimator
{
  public:
    // Constructor
    WheelEstimator(PinName PIN_SPEED);
    // Initializer
    void init();
    // Estimate step
    void estimate(float tau = 0.0);
    // Angular displacement [rad] and angular velocity [rad/s] estimations
    float theta_w, omega_w;
  private:
    // Motor hall sensor object
    Hall hall;
    // Angular velocity bias calibration 
    void calibrate();
    // Predict step
    void predict(float tau);
    // Correct step
    void correct();
    // Angular velocity (rad/s) bias
    float b_omega_w;
};

#endif