#ifndef parameters_h
#define parameters_h

#include "cmath"

const float pi = 3.141516;
const float gravity = 9.80665;    // Accelation of gravity [m/s^2]

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
const float dt = 1.0/f;
const float dt_2 = dt/2.0;
const int dt_us = dt*1e6;

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
const float b_ax = -0.0664;
const float b_ay = 0.1602;
const float b_az = 0.5595;
const float f_ax = 1.0065;
const float f_ay = 1.0086;
const float f_az = 0.9925;

// Magnetometer bias and scale factor
const float b_mx = 12.2661;
const float b_my = -23.6147;
const float b_mz = -108.3594;
const float f_mx = 0.9173;
const float f_my = 0.9596;
const float f_mz = 1.1392;
const float f_mxy = 0.0;
const float f_mxz = 0.0;
const float f_myz = 0.0;

/*const float b_mx = 12.6282660081109;
const float b_my = -24.1047301687035;
const float b_mz = -108.7385532950879;
const float f_mx = 0.946418938804894;
const float f_my = 0.975850547374556;
const float f_mz = 1.094620647564040;
const float f_mxy = -0.091650541182414;
const float f_mxz = -0.042612714228236;
const float f_myz = 0.003040479394890;*/

// Acelleration in inertial reference system (m/s^2)
const float ax_I = 0.0;    
const float ay_I = 0.0;
const float az_I = -9.81;

// Magnectic field in inertial reference system (uT)
const float mx_I = 16.665;
const float my_I = 6.602;
const float mz_I = 14.204;

// Gyroscope and measured quaternion error covariance
const float g_cov = 2.7e-6f;
const float q_cov = 1.5e-4f;

#endif