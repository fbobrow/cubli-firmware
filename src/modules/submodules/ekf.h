#ifndef ekf_h
#define ekf_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/utils/matrix.h"

// Extended Kalman Filter class
class EKF
{
  public:
    // Class constructor
    EKF();
    // Prediction and correction steps
    void predict(const Matrix& u);
    void correct(const Matrix& z);
    // State vector x and error covariance matrix P
    Matrix x, P;
  private:
    // State transition function x_dot = f(x,u) and measurement function z = h(x)
    Matrix f(const Matrix& x0, const Matrix& u0);
    Matrix h(const Matrix& x0);
    // State transition matrix A = jacob(f,x), input matrix B = jacob(f,u) and measurement matrix H = jacob(h,x)
    Matrix jacob_f_x(const Matrix& x0, const Matrix& u0);
    Matrix jacob_f_u(const Matrix& x0, const Matrix& u0);
    Matrix jacob_h_x(const Matrix& x0);
    // Normalize quaternion state q = q/norm(q)
    void norm_quat();
    // State noise covariance matrix Q and measurement noise covariance matrix R
    Matrix Q, R;
};

#endif