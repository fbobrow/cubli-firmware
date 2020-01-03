#include "lsm9ds1.h"

// Class constructor 
LSM9DS1::LSM9DS1(PinName sda, PinName scl) : i2c(sda, scl)
{
}

// Initialize sensor
bool LSM9DS1::init()
{
    // Setup I2C bus
    setup_i2c();
    // Test I2C bus
    if (test_i2c()) {
        // Setup gyroscope, accelerometer and magnetometer
        setup_gyr();
        setup_acc();
        setup_mag();
        return true;
    } else {
        return false;
    }
}

// Read sensor data 
void LSM9DS1::read()
{
    // Read accelerometer and gyroscope data
    read_acc();
    read_gyr();
    //read_mag();
}

// Setup I2C bus 
void LSM9DS1::setup_i2c()
{
    // Setup I2C bus frequency to 100kHz
    i2c.frequency(400000);
}

// Test I2C bus 
bool LSM9DS1::test_i2c()
{   
    // Register addresses
    char reg_acc_gyr[1] = {WHO_AM_I};
    char reg_mag[1] = {WHO_AM_I_M};
    // Data that we're going to read
    char data_acc_gyr[1];
    char data_mag[1];

    // Point to register address
    i2c.write(LSM9DS1_ADDRESS_ACC_GYR, reg_acc_gyr, 1);
    // Read data from this address
    i2c.read(LSM9DS1_ADDRESS_ACC_GYR, data_acc_gyr, 1);
    
    // Point to register address
    i2c.write(LSM9DS1_ADDRESS_MAG, reg_mag, 1);
    // Read data from this address
    i2c.read(LSM9DS1_ADDRESS_MAG, data_mag, 1);
    
    // Check if device identity is 0x68 (acc/gyr) and 0x3D (mag)
    if ((data_acc_gyr[0] == 0x68) && (data_mag[0] == 0x3D)) {
        return true;
    } else {
        return false;
    }
}

// Setup gyroscope configurations (full-scale range) 
void LSM9DS1::setup_gyr(gyr_scale g_scale)
{
    // Register address and data that will be writed
    char reg_data[2] = {CTRL_REG1_G, (uint8_t) ((0b011 << 5) | (g_scale << 3) | 0b000)};
    
    // Point to register address and write data
    i2c.write(LSM9DS1_ADDRESS_ACC_GYR, reg_data, 2);

    // Adjust resolution [rad/s / bit] accordingly to choose scale
    switch (g_scale) {
        case GYR_SCALE_245DPS:
            g_res = 8.75f;
            break;
        case GYR_SCALE_500DPS:
            g_res = 17.50f;
            break;
        case GYR_SCALE_2000DPS:
            g_res = 70.0f;
            break;
    }
    // Convert resolution to SI (mdps / bit -> rad/s / bit)
    g_res = (g_res*1.0e-3f)*pi/180.0f;
}

// Setup accelerometer configurations (full-scale range) 
void LSM9DS1::setup_acc(acc_scale a_scale)
{
    // Register address and data that will be writed
    char reg_data[2] = {CTRL_REG6_XL, (uint8_t) ((0b011 << 5) | (a_scale << 3) | 0b000)};
    
    // Point to register address and write data
    i2c.write(LSM9DS1_ADDRESS_ACC_GYR, reg_data, 2);

    // Adjust resolution [mg / bit] accordingly to choosed scale
    switch (a_scale) {
        case ACC_SCALE_2G:
            a_res = 0.061f;
            break;
        case ACC_SCALE_4G:
            a_res = 0.122f;
            break;
        case ACC_SCALE_8G:
            a_res = 0.244f;
            break;
        case ACC_SCALE_16G:
            a_res = 0.732f;
            break;
    }
    // Convert resolution to SI (mg / bit -> m/s^2 / bit)
    a_res = (a_res*1.0e-3f)*g;
}

// Setup magnetometer configurations (full-scale range) 
void LSM9DS1::setup_mag(mag_scale m_scale)
{
    // Register address and data that will be writed
    char cmd[4] = {CTRL_REG1_M, 0x10, (uint8_t) (m_scale << 5), 0 };

    // Write the data to the mag control registers
    i2c.write(LSM9DS1_ADDRESS_MAG, cmd, 4);

    // Adjust resolution [mgauss / bit] accordingly to choosed scale
    switch (m_scale) {
        case MAG_SCALE_4G:
            m_res = 0.14f;
            break;
        case MAG_SCALE_8G:
            m_res = 0.29f;
            break;
        case MAG_SCALE_12G:
            m_res = 0.43f;
            break;
        case MAG_SCALE_16G:
            m_res = 0.58f;
            break;
    }
    // Convert resolution to SI (mgauss / bit -> uT / bit)
    m_res = ((m_res*1.0e-3f)*1.0e-4f)*1e6f;
}

// Read gyroscope data 
void LSM9DS1::read_gyr()
{
    // LSM9DS1 I2C bus address
    char address = LSM9DS1_ADDRESS_ACC_GYR;
    // Register address
    char reg[1] = {OUT_X_L_G};
    // Data that we're going to read
    char data[6];

    // Point to register address
    i2c.write(address, reg, 1);
    // Read data from this address (register address will auto-increment and all three axis information (two 8 bit data each) will be read)
    i2c.read(address, data, 6);

    // Reassemble the data (two 8 bit data into one 16 bit data)
    int16_t gx_raw = data[0] | ( data[1] << 8 );
    int16_t gy_raw = data[2] | ( data[3] << 8 );
    int16_t gz_raw = data[4] | ( data[5] << 8 );
    // Convert to SI units [rad/s]
    gx = gy_raw * g_res;
    gy = gx_raw * g_res;
    gz = gz_raw * g_res;
}

// Read accelerometer output data 
void LSM9DS1::read_acc()
{
    // LSM9DS1 I2C bus address
    char address = LSM9DS1_ADDRESS_ACC_GYR;
    // Register address
    char reg[1] = {OUT_X_L_XL};
    // Data that we're going to read
    char data[6];

    // Point to register address
    i2c.write(address, reg, 1);
    // Read data from this address (register address will auto-increment and all three axis information (two 8 bit data each) will be read)
    i2c.read(address, data, 6);

    // Reassemble the data (two 8 bit data into one 16 bit data)
    int16_t ax_raw = data[0] | ( data[1] << 8 );
    int16_t ay_raw = data[2] | ( data[3] << 8 );
    int16_t az_raw = data[4] | ( data[5] << 8 );
    // Convert to SI units [m/s^2]
    ax = -ay_raw * a_res;
    ay = -ax_raw * a_res;
    az = -az_raw * a_res;
}

// Read magnetometer output data 
void LSM9DS1::read_mag()
{
    // LSM9DS1 I2C bus address
    char address = LSM9DS1_ADDRESS_MAG;
    // Register address
    char reg[1] = {OUT_X_L_M};
    // Data that we're going to read
    char data[6];

    // Point to register address
    i2c.write(address, reg, 1);
    // Read data from this address (register address will auto-increment and all three axis information (two 8 bit data each) will be read)
    i2c.read(address, data, 6);

    // Reassemble the data (two 8 bit data into one 16 bit data)
    int16_t mx_raw = data[0] | ( data[1] << 8 );
    int16_t my_raw = data[2] | ( data[3] << 8 );
    int16_t mz_raw = data[4] | ( data[5] << 8 );
    // Convert to SI units [uT]
    mx = my_raw * m_res;
    my = -mx_raw * m_res;
    mz = mz_raw * m_res;
}