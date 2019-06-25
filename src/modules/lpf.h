#ifndef lpf_h
#define lpf_h

#include "mbed.h"
#include "utils/matrix.h"

// Low pass filter class
class LowPassFilter
{
  public:
    // Class constructor
    LowPassFilter(float freq, float omega_c);
    // Update step
    void update(const Matrix& u);
    // Output
    Matrix y;
  private:
    // Smoothing factor
    float alpha;
};

#endif