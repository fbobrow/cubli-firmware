# Cubli Firmware  

This project contains the source code for the firmware used in the Cubli project of a PhD Thesis.

### Crazyflie 1.0 support

The 2017.06 release was the last release with Crazyflie 1.0 support. If you want
to play with the Crazyflie 1.0 and modify the code, please clone this repo and
branch off from the 2017.06 tag. 

## Dependencies

You'll need to use either the [Crazyflie VM](https://wiki.bitcraze.io/projects:virtualmachine:index),
[the toolbelt](https://wiki.bitcraze.io/projects:dockerbuilderimage:index) or 
install some ARM toolchain.

### Install a toolchain

#### OS X
```bash
brew tap PX4/homebrew-px4
brew install gcc-arm-none-eabi
