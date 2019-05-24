#ifndef pin_names_h
#define pin_names_h

#include "mbed.h"

// Motor pins
const PinName M1_EN = A0; 
const PinName M2_EN = D11;
const PinName M3_EN = A3; 
const PinName M1_SPEED = A1;
const PinName M2_SPEED = D3;
const PinName M3_SPEED = A4;
const PinName M1_CURRENT = A2;
const PinName M2_CURRENT = D10;
const PinName M3_CURRENT = A5;

// I2C pins
const PinName IMU_SDA = D4; 
const PinName IMU_SCL = D5;

#endif