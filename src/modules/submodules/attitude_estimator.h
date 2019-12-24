#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"
#include "src/drivers/lsm9ds1.h"

// Speed estimator class
class AttitudeEstimator
{
  public:
    // Constructor
    AttitudeEstimator();
    // Initializer
    void init();
    // Estimate step
    void estimate();
    // Angular displacement (rad) and angular velocity (rad/s) estimations
    float theta_s, omega_s;
    float theta_s_m, omega_s_m;
  private:
    // Motor hall sensor object
    LSM9DS1 imu;
    // Angular velocity (rad/s) bias
    float b_omega_s;
    // Angular velocity bias calibration 
    void calibrate();
};

#endif