#include "escon.h"

Escon::Escon(PinName PIN_EN, PinName PIN_SPEED, PinName PIN_CURRENT) : en(PIN_EN), speed(PIN_SPEED), current(PIN_CURRENT)
{
    //current.period_ms(2);
}

// Read speed
float Escon::read()
{
    return speed.read();
}

// Set current
void Escon::set(float i)
{
    if (i == 0.0f)
    {
        en = false;
        current = 0.5f;   
    }
    else
    {
        en = true;
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
            //current = (i+stall_current)/(2.0f*stall_current);   
            current = 0.5+i*(0.8f/(2.0f*stall_current)); 
        }
    }
}