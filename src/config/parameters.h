#ifndef parameters_h
#define parameters_h

#include "cmath"
#include "src/utils/matrix.h"

// Physical parameters
const float pi = 3.141516;                 
const float g = 9.80665;                    // Accelation of gravity [m/s^2]

// Motor parameters
const float Ra = 1.03;                      // Armature resistance []
const float La = 0.572e-3;                  // Armature inductance [H]
const float Km = 33.5e-3;                   // Torque constant [N.m/A]
const float ia_max = 15.0;                  // Stall current [A]
const float omega_nl = 6710.0*(2.0*pi/60.0);// No load speed [rpm -> rad/]

// Reaction wheel friction parameters
const float tau_c = 2.46e-3;                // Coulomb friction torque [N.m]
const float b = 1.06e-5;                    // Rotational viscuous friction coefficient [N.m.s/rad]
const float kd = 1.70e-8;                   // Rotational drag coefficient [N.m.s^2/rad^2]

// Structure wheel parameters
const float I_w = 1.25e-4;                  // Reaction wheel moment of inertia at center of mass [kg.m^2]
const float I_c = 1.00e-2;                  // Cubli total moment of inertia at pivot point [kg.m^2]
const float m_c = 0.85;                     // Cubli total mass [kg]
const float l = 0.15;                       // Cubli side length [m]
const float d = l*sqrt(2.0)/2.0;            // Distance from pivot point to center of mass [m]

// Interrupt frequencies
const float f = 500.0;                      // Controller interrupt frequency [Hz]
const float f_blink = 1.0;                  // Led blink interrupt frequency [Hz]
const float f_print = 10.0;                 // Serial print interrupt frequency [Hz]
const float dt = 1.0/f;
const float dt_blink = 1.0/f_blink;
const float dt_print = 1.0/f_print;
const int dt_us = dt*1e6;
const int dt_blink_us = dt_blink*1e6;
const int dt_print_us = dt_print*1e6;

const float dt_2 = dt/2.0;
const float dt2_2 = pow(dt,2)/2.0; 

// Controller gains
const float alpha = 0.07;
const float zeta = sqrt(2.0)/2.0;
const float wn = sqrt(m_c*g*d/(I_c-I_w)); 
const float kps = pow(wn,2)*(1.0+pow(alpha,2)+4.0*alpha*zeta)-pow(alpha,2)*pow(wn,4)*(I_c-I_w)/(m_c*d*g);
const float kds = 2.0*wn*(alpha+zeta)-2.0*alpha*pow(wn,3)*(1.0+alpha*zeta)*(I_c-I_w)/(m_c*d*g);
const float kpw = pow(alpha,2)*pow(wn,4)*I_w/(m_c*d*g);
const float kdw = 2.0*alpha*pow(wn,3)*(1.0+alpha*zeta)*I_w/(m_c*d*g);

// Estimator gains
const float lps = 1.0;
const float lds = 200.0;
const float lpw = 0.0;
const float ldw = 50.0;


/* XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX */

// Gyroscope bias
const float b_gx = 0.0994;
const float b_gy = 0.0089;
const float b_gz = 0.0289;

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
const float a_cov = 2.7e-4f;
const float q_cov = 1.5e-4f;

#endif