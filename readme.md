# Cubli Firmware  

This repository contains the source code for the firmware utilized in the Cubli project of a PhD Thesis. It was developed with [Mbed OS](https://www.mbed.com/en/platform/mbed-os/) open-source operating system for platforms using ARM microcontrollers.

## Importing

This project was developed in [Mbed Studio](https://os.mbed.com/studio/), however you can also import it to [Mbed Online Compiler](https://ide.mbed.com/) or some other IDE.

### Mbed Studio

1. Install [Mbed Studio](https://os.mbed.com/studio/)
2. Open Mbed Studio
3. Click on ```File```>```Import Program...```
4. Paste ```https://github.com/fbobrow/cubli-firmware/``` under ```URL``` and then click ```Add Program```

### Mbed Online Compiler

1. Open [Mbed Online Compiler](https://ide.mbed.com/)
2. Click on ```Import```
3. Click on ```Click here to import from URL```
4. Paste ```https://github.com/fbobrow/cubli-firmware/``` under ```Source URL:``` and then click ```Import```

## Compiling

The project was developed for the [NUCLEO-L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) development board from [STMicroeletronics](https://www.st.com/content/st_com/en.html). 
*** Main System-on-Chip: STM32F405RG
CPU: 168 MHz ARM Cortex M4 with single-precision FPU
RAM: 192 KB SRAM
nRF51822 radio and power management MCU
MPU9250 Accel / Gyro / Mag
LPS25H barometer

You should choose this board as ```target``` or adapt the project according to the chosen board.
