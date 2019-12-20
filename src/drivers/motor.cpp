#include "motor.h"

// Class constructor
Motor::Motor(PinName PIN_ENABLE, PinName PIN_CURRENT) : enable(PIN_ENABLE), current(PIN_CURRENT)
{
    current.period_ms(1);
}

// Set current [A]
void Motor::set_current(float ia)
{
    if (ia == 0.0)
    {
        enable = false;
        current = 0.5;   
    }
    else
    {
        enable = true;
        if(ia > ia_max)
        {
            current = 0.9;
        }
        else if (ia < -ia_max)
        {
            current = 0.1;
        }
        else 
        {
            current = 0.5+ia*(0.8/(2.0*ia_max)); 
        }
    }
}

// Set torque [N.m]
void Motor::set_torque(float tau)
{
    set_current(tau/Km);
}