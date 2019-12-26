#ifndef ahrs_h
#define ahrs_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"
#include "src/drivers/lsm9ds1.h"
#include "src/utils/matrix.h"
#include "src/modules/ahrs/ekf.h"
#include "src/modules/ahrs/triad.h"

// Attitude and heading eference system
class AHRS
{
  public:
    // Class constructor
    AHRS();
    // Class initializer
    void init();
    // Update step
    void update();
    // Orientation quaternion and angular velocity vector
    Matrix q, omega;
  private:
    // IMU object
    LSM9DS1 imu;
    // Extended Kalman filter object
    EKF ekf;
    // Triad object
    Triad triad;
};

#endif