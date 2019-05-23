#ifndef attitude_estimator_h
#define attitude_estimator_h

#include "mbed.h"
#include "utils/matrix.h"
#include "modules/ekf.h"
#include "modules/triad.h"

// Acelerometer bias and scale factor
#define b_ay 0.0671f
#define b_ax 0.0846f
#define b_az 0.5557f
#define f_ay 1.0080f
#define f_ax 1.0060f
#define f_az 0.9930f

// Magnetometer bias and scale factor
#define b_my -20.6220f
#define b_mx 16.1000f
#define b_mz -106.4630f
#define f_my 1.0611f
#define f_mx 1.0133f
#define f_mz 0.9340f

// Gyroscope and measured quaternion error covariance
#define g_cov 2.7e-6f
#define q_cov 1.5e-4f

// Attitude estimator class
class AttitudeEstimator
{
  public:
    // Class constructor
    AttitudeEstimator(float freq);
    // Update step
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    // Orientation quaternion and angular velocity vector
    Matrix q, omega;
  private:
    // Extended Kalman Filter class
    ExtendedKalmanFilter ekf;
    // Triad class
    Triad triad;
};

#endif