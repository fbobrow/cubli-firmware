# Cubli Firmware  

This repository contains the source code for the firmware utilized in the Cubli project of a PhD Thesis. 

It was developed with [ARM Mbed OS](https://www.mbed.com/en/platform/mbed-os/) open-source operating system targeting the [STM32 NUCLEO-L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) development board:
* 80MHz ARM®32-bit Cortex®M4
* 256 KB Flash
* 64 KB SRAM

For Cubli attitude estimation it utilizes the [SparkFun 9Dof Sensor Stick](https://www.sparkfun.com/products/13944) inertial measurement unit (IMU) composed by the [ST LSM9DS1](https://www.st.com/en/mems-and-sensors/lsm9ds1.html) motion-sensing system-in-a-chip:
* 3D gyroscope (±245/±500/±2000 dps)
* 3D accelerometer (±2g/±4g/±8/±16 g)
* I2C serial bus (100 kHz and 400 kHz)

For reaction wheel estimation and control it utilizes the [Maxon EC 45 flat](https://www.maxongroup.com/maxon/view/product/339286) motor together with the [Maxon ESCON Module 50/5](https://www.maxongroup.com/maxon/view/product/control/4-Q-Servokontroller/438725) controller:
* 24V/50W brushless motor with hall sensors
* 50V/5A/250W 4-quadrant PWM controller

All the remaining mechanical and electrical parts (aluminuim structure, PCBs, etc.) were tailor-made in [Autodesk Fusion 360](https://www.autodesk.com/products/fusion-360/overview) and [Autodesk Eagle](https://www.autodesk.com/products/eagle/overview) and can be found in this other [repository](https://github.com/fbobrow/cubli-firmware).

## Source code

The source code is organized as follows:

```
./                                       | Root
 main.cpp                                | Main source code (call one of the programs)
 + programs                              | Programs files
 |   + examples                          | Example programs
     |  + drivers                        | Driver example programs
     |  + modules                        | Modules example programs
 + src                                   | Source files
 |   cubli.h                             | Include all header files
 |  + drivers                            | Drivers files
    |   hall.h/.cpp                      | Hall sensor class [header / source code]
    |   lsm9ds1.h/.cpp                   | IMU sensor class [header / source code]
    |   motor.h/.cpp                     | Motor controller class [header / source code]
 |  + modules                            | Modules files
    |   estimator_speed.h/.cpp           | Speed estimator [header / source code]
    |   estimator_attitude.h/.cpp        | Attitude estimator [header / source code]
    |   controller_attitute_wheel.h/.cpp | Attitude and wheel controller [header / source code]
 |  + utils                              | Utility files
    |   parameters.h                     | Parameters (physical constants, controller gains, etc.)
    |   pin_names.h                      | Pin names
 + mbed-os                               | ARM Mbed OS source files (automatically imported)
```

## Importing

You can import this project directly to [Mbed Studio](https://os.mbed.com/studio/) or [Mbed Online Compiler](https://ide.mbed.com/).

### Mbed Studio

1. Install and open [Mbed Studio](https://os.mbed.com/studio/)
2. Click on ```File```>```Import Program...```
3. Paste ```https://github.com/fbobrow/cubli-firmware/``` under ```URL``` and then click ```Add Program```

### Mbed Online Compiler

1. Open [Mbed Online Compiler](https://ide.mbed.com/)
2. Click on ```Import```>```Click here``` to import from URL
3. Paste ```https://github.com/fbobrow/cubli-firmware/``` under ```Source URL:``` and then click ```Import```

## Building

You should choose the ```NUCLEO-L432KC``` platform as ```Target``` while builging the project or adapt it to your Mbed compatible board.
