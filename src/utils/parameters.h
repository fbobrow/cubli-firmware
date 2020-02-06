#ifndef parameters_h
#define parameters_h

#include "cmath"

// Interrupt frequencies
const float f = 1000.0;                      // Controller interrupt frequency [Hz]
const float f_blink = 1.0;                  // Led blink interrupt frequency [Hz]
const float f_print = 10.0;                 // Serial print interrupt frequency [Hz]
const float dt = 1.0/f;
const float dt_blink = 1.0/f_blink;
const float dt_print = 1.0/f_print;
const int dt_us = dt*1e6;
const int dt_blink_us = dt_blink*1e6;
const int dt_print_us = dt_print*1e6;

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
const float cd = 1.70e-8;                   // Rotational drag coefficient [N.m.s^2/rad^2]

// Structure parameters
const float l = 0.15;                       // Structure side length [m]
const float m_s = 0.40;                     // Structure mass [kg]
const float I_s_xx = 2.0e-3;                // Structure moment of inertia around x-y-z axis at center of mass [kg.m^2]

// Reaction wheel parameters
const float m_w = 0.15;                     // Reaction wheel mass [kg]
const float I_w_xx = 1.25e-4;               // Reaction wheel moment of inertia around x axis at center of mass [kg.m^2]
const float I_w_yy = 4.0e-5;                // Reaction wheel moment of inertia around y-z axis at center of mass [kg.m^2]

// Cubli (structure + reaction wheels) parameters
const float m_c = m_s+3*m_w;                // Cubli total mass [kg]
const float I_c_xx = I_s_xx+I_w_xx+2*I_w_yy+(m_s+2.0*m_w)*l*l/2.0; // Cubli moment of inertia around x-y-z axis at pivot point [kg.m^2]
const float I_c_xy = -(m_s+m_w)*l*l/4.0;    // Cubli product of inertia at pivot point [kg.m^2]

//
const float m_c_bar = m_c - m_w;            //
const float I_c_xx_bar = I_c_xx - I_w_xx;   //
const float I_c_xy_bar = I_c_xy;            //

// Estimator gains
const float lds = 2.0;
const float ldw = 50.0;

// Quaternion reference (Cubli in vertex fancing up minus phi_e - corresponding to center os mass disalignment)
const float phi_e = -0*pi/180.0;
const float qr0 =                cos(phi_e/2.0 + acos(sqrt(3.0)/3.0)/2.0);
const float qr1 =  sqrt(2.0)/2.0*sin(phi_e/2.0 + acos(sqrt(3.0)/3.0)/2.0);
const float qr2 = -sqrt(2.0)/2.0*sin(phi_e/2.0 + acos(sqrt(3.0)/3.0)/2.0);
const float qr3 =  0.0;

// Quaternion reference (Cubli in x-edge minus phi_e - corresponding to center os mass disalignment)
/*const float phi_e = -5*pi/180.0;
const float qr0 = cos(phi_e/2.0 -     pi/8.0);
const float qr1 = cos(phi_e/2.0 + 3.0*pi/8.0);
const float qr2 = 0.0;
const float qr3 = 0.0;*/

// Quaternion reference (Cubli in y-edge minus phi_e - corresponding to center os mass disalignment)
/*const float phi_e = 3.0*pi/180.0;
const float qr0 = cos(phi_e/2.0 -     pi/8.0);
const float qr1 = 0.0;
const float qr2 = -cos(phi_e/2.0 + 3.0*pi/8.0);
const float qr3 = 0.0;*/

// 
const float phi_min = 10.0*pi/180.0;
const float phi_max = 30.0*pi/180.0;

// Stable quaternion (Cubli vertex facing down)
const float qs0 =                sin(acos(sqrt(3.0)/3.0)/2.0);
const float qs1 = -sqrt(2.0)/2.0*cos(acos(sqrt(3.0)/3.0)/2.0);
const float qs2 =  sqrt(2.0)/2.0*cos(acos(sqrt(3.0)/3.0)/2.0);
const float qs3 =  0.0;
                                   
//
const float m_c_bar_g_l = m_c_bar*g*l;
const float omega_0 = sqrt(m_c_bar_g_l*(sqrt(3.0)/2.0)/(I_c_xx_bar-I_c_xy_bar));
const float delta = m_c_bar_g_l*(sqrt(3.0)/2.0)/I_w_xx;

// Controller gains
/*const float alpha = 0.05;
const float zeta = sqrt(2.0)/2.0;
const float omega_n = omega_0;
const float kpw = pow(alpha,2)*pow(zeta,2)*pow(omega_n,4)/delta;
const float kdw = 2.0*alpha*zeta*pow(omega_n,3)*(1.0+alpha*pow(zeta,2))/delta;
const float kp = pow(omega_n,2)*(1.0+alpha*pow(zeta,2)*(4.0+alpha))-I_c_xx_bar/I_w_xx*kpw;
const float kd = 2.0*zeta*omega_n*(1.0+alpha)-I_c_xx_bar/I_w_xx*kdw;*/


const float zeta = /*1.0;*/sqrt(2.0)/2.0;
const float omega_n = /*2.0*omega_0;*/1.5*omega_0;
const float zeta_w = /*1.0;*/sqrt(2.0)/2.0;
const float omega_n_w = omega_0/10.0;
const float kpw = (pow(omega_n,2)*pow(omega_n_w,2))/delta;
const float kdw = (2.0*zeta_w*pow(omega_n,2)*omega_n_w + 2.0*zeta*omega_n*pow(omega_n_w,2))/delta;
const float kp = (pow(omega_n,2) + 4.0*zeta_w*zeta*omega_n*omega_n_w + pow(omega_n_w,2))-I_c_xx_bar/I_w_xx*kpw;
const float kd = (2.0*omega_n*zeta + 2.0*omega_n_w*zeta_w)-I_c_xx_bar/I_w_xx*kdw;


// Acelerometer bias and scale factor
const float b_ax = -0.0664;
const float b_ay = 0.1602;
const float b_az = 0.5595;
const float f_ax = 1.0065;
const float f_ay = 1.0086;
const float f_az = 0.9925;

#endif