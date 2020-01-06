#ifndef parameters_h
#define parameters_h

#include "cmath"

// Physical parameters
const float pi = 3.141516;                 
const float g = 9.80665;                    // Accelation of gravity [m/s^2]

// Motor electrical parameters
const float Ra = 1.03;                      // Armature resistance []
const float La = 0.572e-3;                  // Armature inductance [H]
const float Km = 33.5e-3;                   // Torque constant [N.m/A]
const float ia_max = 15.0;                  // Stall current [A]
const float omega_nl = 6710.0*(2.0*pi/60.0);// No load speed [rpm -> rad/s]

// Motor mechanical parameters
const float tau_c = 2.46e-3;                // Coulomb friction torque [N.m]
const float b = 1.06e-5;                    // Rotational viscuous friction coefficient [N.m.s/rad]
const float kd = 1.70e-8;                   // Rotational drag coefficient [N.m.s^2/rad^2]

// Structure parameters
const float l = 0.15;                       // Structure side length [m]
const float m_s = 0.40;                     // Structure mass [kg]
const float I_s_xx = 2.0e-3;                // Structure moment of inertia around x-y-z axis at center of mass [kg.m^2]

// Reaction wheel parameters
const float m_w = 0.15;                     // Reaction wheel mass [kg]
const float I_w_xx = 1.25e-4;               // Reaction wheel moment of inertia around x axis at center of mass [kg.m^2]
const float I_w_yy = 4.0e-5;                // Reaction wheel moment of inertia around y-z axis at center of mass [kg.m^2]

// Cubli (structure + reaction wheels) parameters
const float m_c = m_s+3*m_w;                         // Cubli total mass [kg]
const float I_w = I_w_xx;                            // Reaction wheel moment of inertia
const float I_c = I_s_xx+2*I_w_yy+l*l*(m_s/2.0+m_w); // Cubli moment of inertia around x-y-z axis at pivot point [kg.m^2]
const float I_c_pi = -l*l*(m_s+m_w)/4.0;             // Cubli product of inertia at pivot point [kg.m^2]

// Interrupt frequencies
const float f = 200.0;                      // Controller interrupt frequency [Hz]
const float f_blink = 1.0;                  // Led blink interrupt frequency [Hz]
const float f_print = 10.0;                 // Serial print interrupt frequency [Hz]
const float dt = 1.0/f;
const float dt_blink = 1.0/f_blink;
const float dt_print = 1.0/f_print;
const int dt_us = dt*1e6;
const int dt_blink_us = dt_blink*1e6;
const int dt_print_us = dt_print*1e6;

// Estimator gains
const float lds = 2.0;
const float ldw = 50.0;

// Controller gains
const float alpha = 0.07;
const float zeta = sqrt(2.0)/2.0;
const float wn = sqrt(m_c*g*l*sqrt(2.0)/2.0/I_c); 
const float kps = pow(wn,2)*(1.0+pow(alpha,2)+4.0*alpha*zeta)-pow(alpha,2)*pow(wn,4)*I_c/(m_c*l*sqrt(2.0)/2.0*g);
const float kds = 2.0*wn*(alpha+zeta)-2.0*alpha*pow(wn,3)*(1.0+alpha*zeta)*I_c/(m_c*l*sqrt(2.0)/2.0*g);
const float kpw = pow(alpha,2)*pow(wn,4)*I_w/(m_c*l*sqrt(2.0)/2.0*g);
const float kdw = 2.0*alpha*pow(wn,3)*(1.0+alpha*zeta)*I_w/(m_c*l*sqrt(2.0)/2.0*g);

// Acelerometer bias and scale factor
const float b_ax = -0.0664;
const float b_ay = 0.1602;
const float b_az = 0.5595;
const float f_ax = 1.0065;
const float f_ay = 1.0086;
const float f_az = 0.9925;

#endif