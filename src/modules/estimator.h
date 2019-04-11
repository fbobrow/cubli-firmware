#ifndef estimator_h
#define estimator_h

#include "mbed.h"
#include "modules/ekf.h"
#include "modules/triad.h"
#include "utils/matrix.h"

#define b_ax 0.0671f
#define b_ay 0.0846f
#define b_az -0.5557f
#define f_ax 1.0080f
#define f_ay 1.0060f
#define f_az 0.9930f
#define b_mx -20.6220f
#define b_my 16.1000f
#define b_mz 106.4630f
#define f_mx 1.0611f
#define f_my 1.0133f
#define f_mz 0.9340f

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
    //
    Triad triad;
    // Time interval
    float dt, dt_2;
};

#endif