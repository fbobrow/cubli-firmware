# Cubli Firmware  

This repository contains the source code for the firmware utilized in the Cubli project of a PhD Thesis. 

It was developed with [ARM Mbed OS](https://www.mbed.com/en/platform/mbed-os/) open-source operating system targeting the [STM32 NUCLEO-L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) development board:
* 80MHz ARM®32-bit Cortex®M4
* 256 KB Flash
* 64 KB SRAM

For attitude estimation it utilizes the [SparkFun 9Dof Sensor Stick] (https://www.sparkfun.com/products/13944) inertial measurement unit (IMU) that utilizes the [ST LSM9DS1] (https://www.st.com/en/mems-and-sensors/lsm9ds1.html) motion-sensing system-in-a-chip:
* 3D gyroscope (±245/±500/±2000 dps)
* 3D accelerometer (±2g/±4g/±8/±16 g)
* 3D magnetometer (±4/±8/±12/±16 gauss)
* I2C serial bus (100 kHz and 400 kHz)

## Source code

The source code is organized as follows:

```
./                             | Root
 main.cpp                      | Main source code (call one of the programs)
 + programs                    | Programs files
 |   main_attitude.cpp         | Program that implements the attitude estimator
 |   main_calibrate.cpp        | Program that calibrate sensors
 + src                         | Source files
 |  + drivers                  | Drivers files
    |   lsm9ds1.h              | Inertial measurement unit (IMU) header
    |   lsm9ds1.cpp            | Inertial measurement unit (IMU) source code
 |  + modules                  | Modules files
    |   attitude_estimator.h   | Attitude estimator header
    |   attitude_estimator.cpp | Attitude estimator source code
    |   ekf.h                  | Extended Kalman filter header
    |   ekf.cpp                | Extended Kalman filter source code
    |   triad.h                | Triad method header
    |   triad.cpp              | Triad method source code
 |  + utils                    | Utility files
    |   matrix.h               | Matrix class header
    |   matrix.cpp             | Matrix class source code
 + mbed-os                     | ARM Mbed OS source files (automatically imported)
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
