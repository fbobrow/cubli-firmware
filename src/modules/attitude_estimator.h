#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"

#include "src/utils/parameters.h"
#include "src/utils/pin_names.h"
#include "src/drivers/lsm9ds1.h"

// Attitude estimator class
class AttitudeEstimator
{
  public:
    // Constructor
    AttitudeEstimator(PinName PIN_SDA = IMU_SDA, PinName PIN_SCL = IMU_SCL);
    // Initializer
    void init();
    // Estimate step
    void estimate();
    // Rotation quaternion and angular velocity (rad/s) estimations
    float q0, q1, q2, q3, omega_x, omega_y, omega_z;
  private:
    // IMU sensor object
    LSM9DS1 imu;
    // Angular velocity (rad/s) bias
    float b_omega_x, b_omega_y, b_omega_z;
    // Angular velocity bias calibration
    void calibrate();
};

#endif