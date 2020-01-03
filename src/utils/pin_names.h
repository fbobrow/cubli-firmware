#ifndef pin_names_h
#define pin_names_h

#include "mbed.h"

// Motor controller pins
const PinName M1_ENABLE = A3; 
const PinName M2_ENABLE = D11;
const PinName M3_ENABLE = A0; 
const PinName M1_CURRENT = A5;
const PinName M2_CURRENT = D10;
const PinName M3_CURRENT = A2;

// Hall sensor pins
const PinName M1_SPEED = A4;
const PinName M2_SPEED = D3;
const PinName M3_SPEED = A1;

// IMU pins
const PinName IMU_SDA = D4; 
const PinName IMU_SCL = D5;

#endif