#ifndef attitude_estimator2_h
#define attitude_estimator2_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"
#include "src/drivers/lsm9ds1.h"
#include "src/utils/matrix.h"
#include "src/modules/submodules/ekf.h"

// Attitude estimator class
class AttitudeEstimator2
{
  public:
    // Constructor
    AttitudeEstimator2(PinName PIN_SDA = IMU_SDA, PinName PIN_SCL = IMU_SCL);
    // Class initializer
    void init();
    // Update step
    void estimate();
    // Orientation quaternion and angular velocity vector
    Matrix q, omega, P;
  private:
    // IMU object
    LSM9DS1 imu;
    // Extended Kalman filter object
    EKF ekf;
};

#endif