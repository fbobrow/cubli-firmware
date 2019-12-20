#ifndef parameters_h
#define parameters_h

#include "cmath"

const float pi = 3.141516;
const float g = 9.80665;    // Accelation of gravity [m/s^2]

// Motor parameters
const float Ra = 1.03;                       // Armature resistance []
const float La = 0.572e-3;                   // Armature inductance [H]
const float Km = 33.5e-3;                    // Torque constant [N.m/A]
const float ia_max = 15.0;                   // Stall current [A]
const float omega_nl = 6710.0*(2.0*pi/60.0); // No load speed [rpm -> rad/]

// Reaction wheel friction parameters
const float tau_c = 2.5e-3; // Coulomb friction torque [N.m]
const float b = 1.05e-5;    // Rotational viscuous friction coefficient [N.m.s/rad]
const float kd = 1.7e-8;    // Rotational drag coefficient [N.m.s^2/rad^2]

// Interrupt frequencies
const float f = 200.0;
const float dt = 1/200.0;
const int dt_us = (int) dt*1e6;

// Controller gains
const float kps = 0.0;
const float kds = 0.0;
const float kpw = 0.0;
const float kdw = 0.0;

// Estimator gains
const float lpw = 0.0;
const float ldw = 10.0;

const float freq_blink = 1.0f;
const float freq_estimator = 250.0f;
const float freq_controller = 50.0f;

// Motor bias
const float b_omega_1 = -3.5077f;   // rad/s
const float b_omega_2 = -4.0757f;   // rad/s
const float b_omega_3 = -3.9992f;   // rad/s 

//
const float lpf_cut_off_frequency = 100.0f; 

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