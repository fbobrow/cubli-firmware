#ifndef we_h
#define we_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"
#include "src/drivers/hall.h"

// Speed estimator class
class WheelEstimator
{
  public:
    // Constructor
    WheelEstimator(PinName PIN_SPEED);
    // Initializer
    void init();
    // Predict step
    void predict(float tau);
    // Correct step
    void correct();
    // Angular displacement (rad) and angular velocity (rad/s) estimations
    float omega_w, theta_w;
  private:
    // Motor hall sensor object
    Hall hall;
    // Angular velocity (rad/s) bias
    float b_omega_w;
    // Angular velocity bias calibration 
    void calibrate();
};

#endif