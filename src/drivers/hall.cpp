#include "hall.h"

// Class constructor
Hall::Hall(PinName PIN_SPEED) : speed(PIN_SPEED)
{
}

// Read angular velocity
void Hall::read()
{
    omega = (speed.read()-0.5)*(2.0*omega_nl);
}