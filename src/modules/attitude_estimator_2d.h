#ifndef attitude_estimator_2d_h
#define attitude_estimator_2d_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"
#include "src/drivers/lsm9ds1.h"

// Attitude estimator class
class AttitudeEstimator2D
{
  public:
    // Constructor
    AttitudeEstimator2D(PinName PIN_SDA = IMU_SDA, PinName PIN_SCL = IMU_SCL);
    // Initializer
    void init();
    // Estimate step
    void estimate(float tau = 0.0, float omega_w = 0.0);
    // Angular displacement (rad) and angular velocity (rad/s) estimations
    float theta_s, omega_s;
  private:
    // Motor hall sensor object
    LSM9DS1 imu;
    // Angular velocity (rad/s) bias
    float b_omega_s;
    // Angular velocity bias calibration 
    void calibrate();
    // Predict step
    void predict(float tau, float omega_w);
    // Correct step
    void correct();
};

#endif