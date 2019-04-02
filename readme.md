# Cubli Firmware  

This repository contains the source code for the firmware utilized in the Cubli project of a PhD Thesis. 

It was developed with [Mbed OS](https://www.mbed.com/en/platform/mbed-os/) open-source operating system targeting the [NUCLEO-L432KC](https://www.st.com/en/evaluation-tools/nucleo-l432kc.html) development board:
* 80MHz ARM®32-bit Cortex®M4
* 256 KB Flash
* 64 KB SRAM

## Importing

You can import this project directly to [Mbed Studio](https://os.mbed.com/studio/) or [Mbed Online Compiler](https://ide.mbed.com/).

### Mbed Studio

1. Install and open [Mbed Studio](https://os.mbed.com/studio/)
2. Click on ```File```>```Import Program...```
4. Paste ```https://github.com/fbobrow/cubli-firmware/``` under ```URL``` and then click ```Add Program```

### Mbed Online Compiler

1. Open [Mbed Online Compiler](https://ide.mbed.com/)
2. Click on ```Import```>```Click here to import from URL```
3. Paste ```https://github.com/fbobrow/cubli-firmware/``` under ```Source URL:``` and then click ```Import```

## Building

You should choose the ```NUCLEO-L432KC``` platform as ```Target``` while builging the project or adapt it to your Mbed compatible board.
