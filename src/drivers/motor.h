#ifndef motor_h
#define motor_h

#include "mbed.h"

#include "src/config/parameters.h"
#include "src/config/pin_names.h"

// Motor class
class Motor
{
  public:
    // Class constructor
    Motor(PinName PIN_ENABLE = M1_ENABLE, PinName PIN_CURRENT = M1_CURRENT);
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