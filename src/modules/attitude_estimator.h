#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"
#include "drivers/lsm9ds1.h"
#include "utils/matrix.h"

// Attitude estimator class
class AttitudeEstimator
{
  public:
    // Class constructor
    AttitudeEstimator(float frequency);
    // Initialize class
    void init();
    // Estimation step (predict and correct/update with extended Kalman filter)
    void estimate();
    // Quaternion orientation and angular velocity vectors
    Matrix q, omega;
  private:
    //
    void read();
    //
    void update_measurement();
    //
    void update_jacobian();
    //
    Matrix f(Matrix&);
    //
    Matrix h(Matrix&);
    //
    void update_estimated_states();
    // IMU sensor object
    LSM9DS1 imu;
    // Time interval
    float dt, dt_2;
    // State and output vector
    Matrix x, z;
    // Matrices
    Matrix A, B, H, P, Q, R, K;
    // Gyroscope, accelerometer and magnetometer vectors
    Matrix g, a, m;
    //
    Matrix t1, t2, t3, dcm;
};

#endif