#ifndef ahrs_h
#define ahrs_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/utils/matrix.h"
#include "src/modules/submodules/ekf.h"
#include "src/modules/submodules/triad.h"

// Attitude and heading eference system
class AttitudeHeadingReferenceSystem
{
  public:
    // Class constructor
    AttitudeHeadingReferenceSystem(float freq);
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