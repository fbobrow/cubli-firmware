#include "lqr.h"

// Constructor
LinearQuadraticRegulator::LinearQuadraticRegulator()
{
}

// 
void LinearQuadraticRegulator::regulate(float theta_s, float omega_s, float theta_w, float omega_w)
{
    u = kps*(0.0-theta_s)+kds*(0.0-omega_s)+kpw*(0.0-theta_w)+kdw*(0.0-omega_w);
}      