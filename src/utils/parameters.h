#ifndef parameters_h
#define parameters_h

#include "cmath"

// Interrupt frequencies
const float f = 1000.0;                  // Controller interrupt frequency [Hz]
const float f_blink = 1.0;               // Led blink interrupt frequency [Hz]
const float f_print = 10.0;              // Serial print interrupt frequency [Hz]
const float dt = 1.0/f;
const float dt_blink = 1.0/f_blink;
const float dt_print = 1.0/f_print;
const int dt_us = dt*1e6;
const int dt_blink_us = dt_blink*1e6;
const int dt_print_us = dt_print*1e6;

// Acelerometer bias and scale factor
const float b_ax = -0.0664;
const float b_ay = 0.1602;
const float b_az = 0.5595;
const float f_ax = 1.0065;
const float f_ay = 1.0086;
const float f_az = 0.9925;

// Physical parameters
const float pi = 3.141516;                 
const float g = 9.80665;                 // Accelation of gravity [m/s^2]

// Motor (electrical) parameters
const float Ra = 1.03;                   // Armature resistance [Ω]
const float La = 0.572e-3;               // Armature inductance [H]
const float Km = 33.5e-3;                // Torque constant [N.m/A]
const float ia_max = 15.0;               // Stall current [A]
const float omega_nl = 6710.0*(pi/30.0); // No load speed [rpm -> rad/s]

// Motor (mechanical) parameters
const float tau_c = 2.46e-3;             // Coulomb friction torque [N.m]
const float b = 1.06e-5;                 // Rotational viscuous friction coefficient [N.m.s/rad]
const float cd = 1.70e-8;                // Rotational drag coefficient [N.m.s^2/rad^2]

// Structure parameters
const float l = 0.15;                    // Structure side length [m]
const float m_s = 0.40;                  // Structure mass [kg]
const float I_s_xx = 2.0e-3;             // Structure moment of inertia around x-y-z axis at center of mass [kg.m^2]

// Reaction wheel parameters
const float m_w = 0.15;                  // Reaction wheel mass [kg]
const float I_w_xx = 1.25e-4;            // Reaction wheel moment of inertia around x axis at center of mass [kg.m^2]
const float I_w_yy = 4.0e-5;             // Reaction wheel moment of inertia around y-z axis at center of mass [kg.m^2]

// Cubli (structure + reaction wheels) parameters
const float m_c = m_s+3*m_w;             // Cubli total mass [kg]
const float I_c_xx = I_s_xx+I_w_xx+2*I_w_yy+(m_s+2.0*m_w)*l*l/2.0; // Cubli moment of inertia around x-y-z axis at pivot point [kg.m^2]
const float I_c_xy = -(m_s+m_w)*l*l/4.0; // Cubli product of inertia at pivot point [kg.m^2]

// Cubli auxiliary parameters
const float m_c_bar = m_c - m_w;         
const float I_c_xx_bar = I_c_xx - I_w_xx;
const float I_c_xy_bar = I_c_xy;         
const float m_c_bar_g_l = m_c_bar*g*l;
const float omega_0 = sqrt(m_c_bar_g_l*(sqrt(3.0)/2.0)/(I_c_xx_bar-I_c_xy_bar));
const float beta = I_c_xx_bar/I_w_xx;
const float gamma = I_w_xx/(m_c_bar_g_l*(sqrt(3.0)/2.0));
                               
// Estimator gains
const float lds = 2.0;
const float ldw = 50.0;

// Controller gains
const float alpha = 0.05;
const float zeta = sqrt(2.0)/2.0;
const float omega_n = 1.5*omega_0;
const float kpw = gamma*pow(alpha,2)*pow(zeta,2)*pow(omega_n,4);
const float kdw = 2.0*gamma*alpha*zeta*pow(omega_n,3)*(1.0+alpha*pow(zeta,2));
const float kp = pow(omega_n,2)*(1.0+alpha*pow(zeta,2)*(4.0+alpha))+beta*kpw;
const float kd = 2.0*zeta*omega_n*(1.0+alpha)+beta*kdw;
/*const float kpw = 0.0;
const float kdw = gamma*alpha*zeta*pow(omega_n,3);
const float kp = pow(omega_n,2)*(1.0+2.0*alpha*pow(zeta,2));
const float kd = 2.0*zeta*omega_n*(1.0+alpha/2.0)+beta*kdw;*/

// Quaternion reference (Cubli in vertex fancing up minus phi_e - corresponding to center os mass disalignment)
const float phi_e = -0.0*pi/180.0;
const float qu0 =                cos(phi_e/2.0 + acos(sqrt(3.0)/3.0)/2.0);
const float qu1 =  sqrt(2.0)/2.0*sin(phi_e/2.0 + acos(sqrt(3.0)/3.0)/2.0);
const float qu2 = -sqrt(2.0)/2.0*sin(phi_e/2.0 + acos(sqrt(3.0)/3.0)/2.0);
const float qu3 =  0.0;

// Quaternion reference (Cubli in x-edge minus phi_e - corresponding to center os mass disalignment)
/*const float phi_e = -5*pi/180.0;
const float qu0 = cos(phi_e/2.0 -     pi/8.0);
const float qu1 = cos(phi_e/2.0 + 3.0*pi/8.0);
const float qu2 = 0.0;
const float qu3 = 0.0;*/

// Quaternion reference (Cubli in y-edge minus phi_e - corresponding to center os mass disalignment)
/*const float phi_e = 3.0*pi/180.0;
const float qu0 = cos(phi_e/2.0 -     pi/8.0);
const float qu1 = 0.0;
const float qu2 = -cos(phi_e/2.0 + 3.0*pi/8.0);
const float qu3 = 0.0;*/

// Minimum and maximum error limits (for control safety)
const float phi_min = 10.0*pi/180.0;
const float phi_max = 30.0*pi/180.0;

// Trajectory parameters
const float pos_traj = 2.0*pi;          // Trajectory path [rad]
const float t_rest = 5.0;               // Rest time [s]
const float t_traj = 20.0;              // Trajectory time [s]
const float crackle_0 = 720.0*pos_traj/pow(t_traj,5);
const float snap_0 = 360.0*pos_traj/pow(t_traj,4);
const float jerk_0 = 60.0*pos_traj/pow(t_traj,3);

#endif