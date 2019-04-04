#ifndef ekf_h
#define ekf_h

#include "mbed.h"
#include "utils/matrix.h"

// Attitude estimator class
class ExtendedKalmanFilter
{
  public:
    // Class constructor
    ExtendedKalmanFilter(float freq);
    // Estimation step (predict and correct/update with extended Kalman filter)
    void predict(Matrix& u);
    void correct(Matrix& z);
    // State vector
    Matrix x;
  private:
    //
    void update_A(Matrix& u);
    void update_B();
    //
    Matrix f(Matrix& x, Matrix& u);
    Matrix h(Matrix& x);
    //
    void update_estimated_states();
    // Time interval
    float dt, dt_2;
    // Matrices
    Matrix A, B, H, P, Q, R, K;
};

#endif