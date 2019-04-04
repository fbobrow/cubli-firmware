#ifndef estimator_h
#define estimator_h

#include "mbed.h"
#include "modules/ekf.h"
#include "utils/matrix.h"

// Attitude estimator class
class Estimator
{
  public:
    // Class constructor
    Estimator(float freq);
    // Estimation step (predict and correct/update with extended Kalman filter)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    //
    Matrix q, omega;
  private:
    //
    ExtendedKalmanFilter ekf;
    // Time interval
    float dt, dt_2;
    //
    float f_ax, f_ay, f_az;
    float b_ax, b_ay, b_az;
    //
    float f_mx, f_my, f_mz;
    float b_mx, b_my, b_mz;
};

#endif