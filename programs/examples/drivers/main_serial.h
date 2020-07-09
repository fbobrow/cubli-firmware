#include "mbed.h"

// Objects
DigitalOut led(LED1);
Serial pc(SERIAL_TX, SERIAL_RX, NULL, 115200);
Ticker tic_blink;


// Interrupt flags and callback functions
bool flag_blink = false;
void callback_blink() { flag_blink = true; }

// Serial commands
char command;
float x[256][16];

FlashIAP flash;
const uint32_t size = sizeof(x);
const uint32_t addr = flash.get_flash_start()+flash.get_flash_size()-size;

// Main program
int main() 
{
    // Initializations
    tic_blink.attach_us(&callback_blink, 1e6);
    for(int i=0;i<=20;i++)
    {
        wait_us(1e5);
        led = !led;
    }
    // Endless loop
    while (true) 
    {
        if (flag_blink) 
        {
            flag_blink = false;
            led = !led;
        }
        if (pc.readable()) 
        {
            command = pc.getc();
            if (command == 'i') 
            {
                for(int r=0;r<256;r++)
                {
                    for (int c=0;c<16;c++)
                    {
                        x[r][c] = (r+1)*(c+1);
                    }
                }
            }
            if (command == 'p') 
            {
                for(int r=0;r<256;r++)
                {
                    for (int c=0;c<16;c++)
                    {
                        pc.printf("%.4f\t",x[r][c]);
                    }
                    pc.printf("\n");
                }
                pc.printf("\n");
            }
            if (command == 's') 
            {
                flash.init();
                flash.erase(addr, size);
                flash.program(x, addr, size);   
                flash.deinit(); 
                pc.printf("Saved!");  
            }
            if (command == 'r') 
            {
                flash.init();
                flash.read(x, addr, size);
                flash.deinit();
                pc.printf("Read!");  
            }
        }
    }
}

