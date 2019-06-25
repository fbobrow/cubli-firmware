#include "escon.h"

Escon::Escon(PinName PIN_ENABLE, PinName PIN_SPEED, PinName PIN_CURRENT) : enable(PIN_ENABLE), speed(PIN_SPEED), current(PIN_CURRENT)
{
    current.period_ms(1);
}

// Read speed
void Escon::read()
{
    omega = (speed.read()-0.5f)*(2.0f*no_load_speed);
}

void Escon::set_torque(float tau)
{
    set_current(tau/torque_constant);
}

// Set current
void Escon::set_current(float i)
{
    if (i == 0.0f)
    {
        enable = false;
        current = 0.5f;   
    }
    else
    {
        enable = true;
        if(i > stall_current)
        {
            current = 0.9f;
        }
        else if (i < -stall_current)
        {
            current = 0.1f;
        }
        else 
        {
            current = 0.5f+i*(0.8f/(2.0f*stall_current)); 
        }
    }
}