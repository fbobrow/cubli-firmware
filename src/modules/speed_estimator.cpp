#include "speed_estimator.h"

// Class constructor
SpeedEstimator::SpeedEstimator(float freq) : lpf(freq,lpf_cut_off_frequency)
{
    // Initialize xxx
    omega = zeros(3,1);
}

// Update step
void SpeedEstimator::update(float omega_1, float omega_2, float omega_3)
{
    // xxx
    Matrix u(3,1);
    u(1,1) = omega_1-b_omega_1;
    u(2,1) = omega_2-b_omega_2;
    u(3,1) = omega_3-b_omega_3;
    // Low pass filter update step
    lpf.update(u);
    // Triad update
    omega = lpf.y;
}
            