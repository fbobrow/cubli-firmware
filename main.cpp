#include "mbed.h"

DigitalOut led(LED1);

int main() {
    while(1) {
        led = !led; 
        wait(0.5);
    }
}
