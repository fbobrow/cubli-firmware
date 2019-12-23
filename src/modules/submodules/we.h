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
    // Class constructor
    WheelEstimator(PinName PIN_SPEED);
    //
    void init();
    // Predict step
    void predict();
    // Update step
    void correct();
    // XXX
    float theta, omega, omega_m;
  private:
    // Motor hall sensor object
    Hall hall;
    //
    float omega_bias;
    //
    void calibrate();
};

#endif