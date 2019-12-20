#ifndef motor_h
#define motor_h

#include "mbed.h"

#include "src/config/parameters.h"

// Motor class
class Motor
{
  public:
    // Class constructor
    Motor(PinName PIN_ENABLE, PinName PIN_CURRENT);
    // Set current [A]
    void set_current(float ia);
    // Set torque [N.m]
    void set_torque(float tau);
  private:
    // Objects
    DigitalOut enable;
    PwmOut     current; 
};

#endif