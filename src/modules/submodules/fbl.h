#ifndef fbl_h
#define fbl_h

#include "mbed.h"

#include "src/config/parameters.h"

// Speed estimator class
class FeedbackLinearization
{
  public:
    // Constructor
    FeedbackLinearization();
    //
    void linearize(float u, float theta_s, float omega_w);
    // 
    float tau;
};

#endif