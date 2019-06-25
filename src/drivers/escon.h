#ifndef escon_h
#define escon_h

#include "mbed.h"
#include "config/parameters.h"

// Escon class
class Escon
{
  public:
    // Class constructor
    Escon(PinName PIN_ENABLE, PinName PIN_SPEED, PinName PIN_CURRENT);
    // Read speed
    float read_speed();
    // Set current
    void set_current(float i);
  private:
    // Objects
    DigitalOut enable;
    AnalogIn   speed;
    PwmOut     current; 
};

#endif