#ifndef estimator_attitude_h
#define estimator_attitude_h

#include "mbed.h"

#include "src/utils/parameters.h"
#include "src/utils/pin_names.h"
#include "src/drivers/lsm9ds1.h"

// Attitude estimator class
class AttitudeEstimator
{
  public:
    // Constructor
    AttitudeEstimator(PinName PIN_SDA, PinName PIN_SCL);
    // Initializer
    void init();
    // Estimate step
    void estimate();
    // Rotation quaternion estimations
    float q0, q1, q2, q3;
    // Angular velocity (rad/s) estimations
    float omega_x, omega_y, omega_z;
  private:
    // IMU sensor object
    LSM9DS1 imu;
    // Angular velocity bias calibration
    void calibrate();
    // Predict step
    void predict(float omega_x, float omega_y, float omega_z);
    // Correct step
    void correct(float ax, float ay, float az);
    // Rotation quaternion time derivative estimations
    float q0_dot, q1_dot, q2_dot, q3_dot;
    // Angular velocity (rad/s) bias
    float b_omega_x, b_omega_y, b_omega_z;
};

#endif