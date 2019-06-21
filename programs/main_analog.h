#include "mbed.h"

Serial pc(SERIAL_TX, SERIAL_RX,NULL,230400);
AnalogIn analog(D3); 

int main()
{
    float measurement;
    float voltage;
    float dt;
    int i = 0;
    while(1) {
        measurement = analog.read(); 
        voltage = measurement*3.3f;
        dt = pow(10.0,-i);
        wait(dt);
        pc.printf("Meas: %.2f | Voltage: %.2fV | dt: %.3f \n", measurement, voltage, dt);
        i++;
        if(i>3)
        {
            i = 0;
            pc.printf("\n");
        }
    }
}
