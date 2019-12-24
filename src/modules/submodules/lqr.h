#ifndef lqr_h
#define lqr_h

#include "mbed.h"

#include "src/config/parameters.h"

// Speed estimator class
class LinearQuadraticRegulator
{
  public:
    // Constructor
    LinearQuadraticRegulator();
    //
    void regulate(float theta_s, float omega_s, float theta_w, float omega_w);
    // 
    float u;
};

#endif