#ifndef lsm9ds1_h
#define lsm9ds1_h

#include "mbed.h"

// Physical constants
#define GRAVITY 9.80665f
#define PI 3.141592f

// LSM9DS1 I2C bus address
#define LSM9DS1_ADDRESS_ACC_GYR 0x6B << 1 // (0xD6) Shift 1 bit left because mbed utilizes 8-bit addresses and not 7-bit 
#define LSM9DS1_ADDRESS_MAG     0x1E << 1 // (0x3C) Shift 1 bit left because mbed utilizes 8-bit addresses and not 7-bit 

// Device identity  
#define WHO_AM_I     0x0F
#define WHO_AM_I_M   0x0F

// Gyroscope configuration registers addresses
#define CTRL_REG1_G  0x10
// Gyroscope output register addresses
#define OUT_X_L_G    0x18
#define OUT_X_H_G    0x19
#define OUT_Y_L_G    0x1A
#define OUT_Y_H_G    0x1B
#define OUT_Z_L_G    0x1C
#define OUT_Z_H_G    0x1D

// Accelerometer configuration registers addresses
#define CTRL_REG6_XL 0x20
// Accelerometer output register addresses
#define OUT_X_L_XL   0x28
#define OUT_X_H_XL   0x29
#define OUT_Y_L_XL   0x2A
#define OUT_Y_H_XL   0x2B
#define OUT_Z_L_XL   0x2C
#define OUT_Z_H_XL   0x2D

// Magnetometer configuration registers addresses
#define CTRL_REG1_M  0x20
#define CTRL_REG2_M  0x21
// Magnetometer output register addresses
#define OUT_X_L_M    0x28
#define OUT_X_H_M    0x29
#define OUT_Y_L_M    0x2A
#define OUT_Y_H_M    0x2B
#define OUT_Z_L_M    0x2C
#define OUT_Z_H_M    0x2D

// Gyroscope full-scale ranges
enum gyr_scale
{
    GYR_SCALE_245DPS = 0b00,   
    GYR_SCALE_500DPS = 0b01,  
    GYR_SCALE_2000DPS = 0b11 
};

// Accelerometer full-scale ranges
enum acc_scale
{
    ACC_SCALE_2G = 0b00, 
    ACC_SCALE_4G = 0b10,
    ACC_SCALE_8G = 0b11, 
    ACC_SCALE_16G = 0b01 
};

// Magnetometer full-scale ranges
enum mag_scale
{
    MAG_SCALE_4G = 0b00,  
    MAG_SCALE_8G = 0b01,
    MAG_SCALE_12G = 0b10,
    MAG_SCALE_16G = 0b11
};

// LSM9DS1 class
class LSM9DS1
{
    public:
    
        // Class constructor
        LSM9DS1(PinName sda, PinName scl);
        
        // Initialize sensor
        bool init();
        // Read sensor data
        void read();
        
        // Gyroscope data in x, y and z axis [rad/s]
        float gx, gy, gz;
        // Accelerometer data x, y and z axis [m/s^2]
        float ax, ay, az;
        // Magnetometer data x, y and z axis [uT]
        float mx, my, mz;
        
    private:
    
        // I2C bus
        I2C i2c;
        
        // Setup I2C bus
        void setup_i2c();
        // Test I2C bus
        bool test_i2c();
        
        // Setup gyroscope configurations (full-scale range)
        void setup_gyr(gyr_scale g_scale = GYR_SCALE_2000DPS);
        // Setup accelerometer configurations (full-scale range)
        void setup_acc(acc_scale a_scale = ACC_SCALE_2G);
        // Setup magnetometer configurations (full-scale range)
        void setup_mag(mag_scale m_scale = MAG_SCALE_4G);
        
        // Read gyroscope data
        void read_gyr();
        // Read accelerometer data
        void read_acc();
        // Read magnetometer data
        void read_mag();
        
        // Gyroscope resolution [rad/s / bit]
        float g_res;
        // Accelerometer resolution [m/s^2 / bit]
        float a_res;
        /// Magnetometers resolution [uT / bit]
        float m_res;
        
};

#endif