#ifndef attitude_estimator_3d_h
#define attitude_estimator_3d_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"
#include "src/drivers/lsm9ds1.h"
#include "src/utils/matrix.h"

// Attitude estimator class
class AttitudeEstimator3D
{
  public:
    // Constructor
    AttitudeEstimator3D(PinName PIN_SDA = IMU_SDA, PinName PIN_SCL = IMU_SCL);
    // Class initializer
    void init();
    // Update step
    void estimate();
    // Orientation quaternion and angular velocity vector
    float q0, q1, q2, q3, omega_x, omega_y, omega_z;
  private:
    // IMU object
    LSM9DS1 imu;
    // Angular velocity (rad/s) bias
    float b_omega_x, b_omega_y, b_omega_z;
    // Angular velocity bias calibration 
    void calibrate();
    // 
    void predict(const Matrix& u);
    //
    void correct(const Matrix& z);
    // State transition function x_dot = f(x,u) and measurement function z = h(x)
    Matrix f(const Matrix& x, const Matrix& u);
    Matrix h(const Matrix& x);
    // State transition matrix A = jacob(f,x), input matrix B = jacob(f,u) and measurement matrix H = jacob(h,x)
    Matrix jacob_f_x(const Matrix& x, const Matrix& u);
    Matrix jacob_f_u(const Matrix& x, const Matrix& u);
    Matrix jacob_h_x(const Matrix& x);
    // State noise covariance matrix Q and measurement noise covariance matrix R
    Matrix x, P;
    Matrix Q, R;
};

#endif