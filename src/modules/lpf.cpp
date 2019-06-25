#include "lpf.h"

// Class constructor
LowPassFilter::LowPassFilter(float freq, float omega_c)
{
    // Initialize output
    y = zeros(3,1);
    // Define smoothing factor
    alpha = omega_c/(freq+omega_c);
}

// Update step
void LowPassFilter::update(const Matrix &u)
{
    y = alpha*u+(1-alpha)*y;
}