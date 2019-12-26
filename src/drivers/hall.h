#ifndef hall_h
#define hall_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"

// Hall class
class Hall
{
  public:
    // Class constructor
    Hall(PinName PIN_SPEED = M1_SPEED);
    // Read angular velocity
    void read();
    // Angular velocity [rad/s]
    float omega;
  private:
    // Objects
    AnalogIn   speed;
};

#endif