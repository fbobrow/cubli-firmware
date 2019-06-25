#ifndef speed_estimator_h
#define speed_estimator_h

#include "mbed.h"
#include "config/parameters.h"
#include "modules/lpf.h"
#include "utils/matrix.h"

// Speed estimator class
class SpeedEstimator
{
  public:
    // Class constructor
    SpeedEstimator(float freq);
    // Update step
    void update(float omega_1, float omega_2, float omega_3);
    // Orientation quaternion and angular velocity vector
    Matrix omega;
  private:
    // Extended Kalman Filter class
    LowPassFilter lpf;
};

#endif