#include "LSM9DS1.h"

/** Class constructor */
LSM9DS1::LSM9DS1(PinName sda, PinName scl) : i2c(sda, scl)
{
}

/** Try to initialize sensor (return true if succeed and false if failed) */
bool LSM9DS1::init()
{
    // Setup I2C bus
    setup_i2c();

    // Test I2C bus
    if (test_i2c()) {
        // Setup accelerometer and gyroscope configurations
        setup_gyr();
        setup_acc();
        setup_mag();
        return true;
    } else {
        return false;
    }
}

/** Read sensor data */
void LSM9DS1::read()
{
    // Read accelerometer and gyroscope data
    read_acc();
    read_gyr();
    read_mag();
}

/** Setup I2C bus */
void LSM9DS1::setup_i2c()
{
    // Setup I2C bus frequency to 100kHz
    i2c.frequency(400000);
}

/** Test I2C bus */
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

/** Setup gyroscope configurations (full-scale range) */
void LSM9DS1::setup_gyr(gyr_scale g_scale)
{
    // Register address and data that will be writed
    char reg_data[2] = {CTRL_REG1_G, (0b011 << 5) | (g_scale << 3) | 0b000};
    
    // Point to register address and write data
    i2c.write(LSM9DS1_ADDRESS_ACC_GYR, reg_data, 2);

    // Adjust resolution [rad/s / bit] accordingly to choose scale
    switch (g_scale) {
        case GYR_SCALE_245DPS:
            g_res = 0.00875f*PI/180.0f;
            break;
        case GYR_SCALE_500DPS:
            g_res = 0.01750f*PI/180.0f;
            break;
        case GYR_SCALE_2000DPS:
            g_res = 0.0700f*PI/180.0f;
            break;
    }
}

/** Setup accelerometer configurations (full-scale range) */
void LSM9DS1::setup_acc(acc_scale a_scale)
{
    // Register address and data that will be writed
    char reg_data[2] = {CTRL_REG6_XL, (0b011 << 5) | (a_scale << 3) | 0b000};
    
    // Point to register address and write data
    i2c.write(LSM9DS1_ADDRESS_ACC_GYR, reg_data, 2);

    // Adjust resolution [m/s^2 / bit] accordingly to choose scale (g)
    switch (a_scale) {
        case ACC_SCALE_2G:
            a_res = 0.000061f*GRAVITY;
            break;
        case ACC_SCALE_4G:
            a_res = 0.000122f*GRAVITY;
            break;
        case ACC_SCALE_8G:
            a_res = 0.000244f*GRAVITY;
            break;
        case ACC_SCALE_16G:
            a_res = 0.000732f*GRAVITY;
            break;
    }
}

/** Setup gyroscope configurations (full-scale range) */
void LSM9DS1::setup_mag(mag_scale m_scale)
{
    // Register address and data that will be writed
    /*char reg_data[2] = {CTRL_REG2_M, (0b0 << 7) | (m_scale << 5) | 0b00000};
    
    // Point to register address and write data
    i2c.write(LSM9DS1_ADDRESS_MAG, reg_data, 2);*/
    
    char cmd[4] = {
        CTRL_REG1_M,
        0x10,       // Default data rate, xy axes mode, and temp comp
        m_scale << 5, // Set mag scale
        0           // Enable I2C, write only SPI, not LP mode, Continuous conversion mode
    };

    // Write the data to the mag control registers
    i2c.write(LSM9DS1_ADDRESS_MAG, cmd, 4);

    // Adjust resolution [gauss / bit] accordingly to choosed scale
    switch (m_scale) {
        case MAG_SCALE_4G:
            m_res = 0.014f;
            break;
        case MAG_SCALE_8G:
            m_res = 0.029f;
            break;
        case MAG_SCALE_12G:
            m_res = 0.043f;
            break;
        case MAG_SCALE_16G:
            m_res = 0.058f;
            break;
    }
}

/** Read gyroscope data */
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
    gx = gx_raw * g_res;
    gy = -gy_raw * g_res;
    gz = gz_raw * g_res;
}

/** Read accelerometer output data */
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
    ax = -ax_raw * a_res;
    ay = ay_raw * a_res;
    az = -az_raw * a_res;
}

/** Read magnetometer output data */
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
    // Convert to SI units [m/s^2]
    mx = -mx_raw * m_res;
    my = -my_raw * m_res;
    mz = mz_raw * m_res;
}