#include "mbed.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 230400);
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
            pc.printf("Hello world!\n");
        }
    }
}