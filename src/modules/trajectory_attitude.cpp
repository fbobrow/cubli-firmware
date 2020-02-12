#include "trajectory_attitude.h"

// Constructor
AttitudeTrajectory::AttitudeTrajectory()
{
    //
    qr0 = qu0;
    qr1 = qu1;
    qr2 = qu2;
    qr3 = qu3;
    //
    omega_r_x = 0.0;
    omega_r_y = 0.0;
    omega_r_z = 0.0;
    //
    alpha_r_x = 0.0;
    alpha_r_y = 0.0;
    alpha_r_z = 0.0;
    //
    flag_rot1 = true;
    flag_res1 = true;
    flag_rot2 = true;
    flag_res2 = true;
}

// Initializer
void AttitudeTrajectory::init()
{
    tim.start();
}

// Generate step
void AttitudeTrajectory::generate()
{
    //
    if ((tim.read() > t_rest/2.0) && flag_rot1)
    {
        flag_rot1 = false;
        crackle = crackle_0;
        snap = -snap_0;
        jerk = jerk_0;
    }
    if ((tim.read() > t_traj+t_rest/2.0) && flag_res1)
    {
        flag_res1 = false;
        jerk = 0.0;
        snap = 0.0;
        crackle = 0.0;
        acc = 0.0;
        vel = 0.0;
    }
    if ((tim.read() > t_traj+3.0*t_rest/2.0) && flag_rot2)
    {
        flag_rot2 = false;
        crackle = -crackle_0;
        snap = snap_0;
        jerk = -jerk_0;
    }
    if ((tim.read() > 2.0*t_traj+3.0*t_rest/2.0) && flag_res2)
    {
        flag_res2 = false;
        jerk = 0.0;
        snap = 0.0;
        crackle = 0.0;
        acc = 0.0;
        vel = 0.0;
    }
    if (tim.read() > 2.0*t_traj+2.0*t_rest)
    {
        tim.reset();
        flag_rot1 = true;
        flag_res1 = true;
        flag_rot2 = true;
        flag_res2 = true;
    }
    //
    vel += acc*dt + jerk*pow(dt,2)/2.0 + snap*pow(dt,3)/6.0 + crackle*pow(dt,4)/24.0;
    acc += jerk*dt + snap*pow(dt,2)/2.0 + crackle*pow(dt,3)/6.0;
    jerk += snap*dt + crackle*pow(dt,2)/2.0;
    snap += crackle*dt; 
    //
    float qr0_dot = 0.5*(-qr1*omega_r_x - qr2*omega_r_y - qr3*omega_r_z);
    float qr1_dot = 0.5*( qr0*omega_r_x - qr3*omega_r_y + qr2*omega_r_z);
    float qr2_dot = 0.5*( qr3*omega_r_x + qr0*omega_r_y - qr1*omega_r_z);
    float qr3_dot = 0.5*(-qr2*omega_r_x + qr0*omega_r_z + qr1*omega_r_y);
    qr0 += qr0_dot*dt;
    qr1 += qr1_dot*dt;
    qr2 += qr2_dot*dt;
    qr3 += qr3_dot*dt;
    //
    omega_r_x = vel/sqrt(3.0);
    omega_r_y = vel/sqrt(3.0);
    omega_r_z = vel/sqrt(3.0);
    //
    alpha_r_x = acc/sqrt(3.0);
    alpha_r_y = acc/sqrt(3.0);
    alpha_r_z = acc/sqrt(3.0);
}
            