#include "mbed.h"
#include "cubli.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
Hall hall(M3_SPEED);
Ticker tic;


// Interrupt flag and callback functions
bool flag = false;
void callback() { flag = true; }

// Main program
int main() 
{
    // Initializations
    tic.attach_us(&callback, 1e6);
    // Endless loop
    while (true) 
    {
        if (flag) 
        {
            flag = false;
            led = !led;
            hall.read();
            pc.printf("Speed (rad/s): %.2f\n",hall.omega);
        }
    }
}