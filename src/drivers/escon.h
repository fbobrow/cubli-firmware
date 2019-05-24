#ifndef escon_h
#define escon_h

#include "mbed.h"
#include "config/parameters.h"

// Escon class
class Escon
{
  public:
    // Class constructor
    Escon(PinName PIN_EN, PinName PIN_SPEED, PinName PIN_CURRENT);
    // Read speed
    float read();
    // Set current
    void set(float i);
  private:
    // Objects
    DigitalOut en;
    AnalogIn   speed;
    PwmOut     current; 
};

#endif