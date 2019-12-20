#ifndef hall_h
#define hall_h

#include "mbed.h"

#include "src/config/parameters.h"

// Hall class
class Hall
{
  public:
    // Class constructor
    Hall(PinName PIN_SPEED);
    // Read angular velocity
    void read();
    // Angular velocity [rad/s]
    float omega;
  private:
    // Objects
    AnalogIn   speed;
};

#endif