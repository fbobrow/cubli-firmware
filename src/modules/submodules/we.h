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
    WheelEstimator();
    //
    void calibrate();
    // Predict step
    void predict();
    // Update step
    void correct();
    // XXX
    float theta, omega;
  private:
    //
    Hall hall;
    //
    float omega_bias;
    //
    float alpha;
};

#endif