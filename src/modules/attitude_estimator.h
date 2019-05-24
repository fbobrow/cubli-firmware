#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"
#include "config/parameters.h"
#include "modules/ekf.h"
#include "modules/triad.h"
#include "utils/matrix.h"

// Attitude estimator class
class AttitudeEstimator
{
  public:
    // Class constructor
    AttitudeEstimator(float freq);
    // Update step
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    // Orientation quaternion and angular velocity vector
    Matrix q, omega;
  private:
    // Extended Kalman Filter class
    ExtendedKalmanFilter ekf;
    // Triad class
    Triad triad;
};

#endif