#ifndef parameters_h
#define parameters_h

#include "cmath"

// Interrupt frequencies
const float freq_blink = 2.0f;
const float freq_estimator = 250.0f;
const float freq_controller = 50.0f;

//
const float stall_current = 15.0f; // 15.0f;
const float no_load_speed = 6710.0f; // 6710.0f*(2.0f*3.141516f/60.0f);

// Acelerometer bias and scale factor
const float b_ay = 0.0671f;
const float b_ax = 0.0846f;
const float b_az = 0.5557f;
const float f_ay = 1.0080f;
const float f_ax = 1.0060f;
const float f_az = 0.9930f;

// Magnetometer bias and scale factor
const float b_my = -20.6220f;
const float b_mx = 16.1000f;
const float b_mz = -106.4630f;
const float f_my = 1.0611f;
const float f_mx = 1.0133f;
const float f_mz = 0.9340f;

// Acelleration in inertial reference system (m/s^2)
const float ax_I = 0.0f;    
const float ay_I = 0.0f;
const float az_I = -9.81f;

// Magnectic field in inertial reference system (uT)
const float mx_I = -6.598f;
const float my_I = 16.730f;
const float mz_I = 14.140f;

// Gyroscope and measured quaternion error covariance
const float g_cov = 2.7e-6f;
const float q_cov = 1.5e-4f;

#endif