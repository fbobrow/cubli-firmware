#include "attitude_wheel_controller_3d.h"

// Constructor
AttitudeWheelController3D::AttitudeWheelController3D()
{
    //
    tau_1 = 0.0;
    tau_2 = 0.0;
    tau_3 = 0.0;
    //
    u_1 = 0.0;
    u_2 = 0.0;
    u_3 = 0.0;
}

// Control step
void AttitudeWheelController3D::control(float q0r, float q1r, float q2r, float q3r, float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float theta_1, float theta_2, float theta_3, float omega_1, float omega_2, float omega_3)
{
    state_regulator(q0r,q1r,q2r,q3r,q0,q1,q2,q3,omega_x,omega_y,omega_z,theta_1,theta_2,theta_3,omega_1,omega_2,omega_3);
    feedback_linearization(q0,q1,q2,q3,omega_x,omega_y,omega_z,omega_1,omega_2,omega_3);
}   

// State regulator step
void AttitudeWheelController3D::state_regulator(float q0r, float q1r, float q2r, float q3r, float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float theta_1, float theta_2, float theta_3, float omega_1, float omega_2, float omega_3)
{
    // State regulator step (with auxiliary variables to avoid double arithmetic) 
    float _2_kps_omegas_4 = 2.0*(kps - (omega_x*omega_x + omega_y*omega_y + omega_z*omega_z)/4.0);
    float q0rq0_q1rq1_q2rq2_q3rq3 = q0r*q0 + q1r*q1 + q2r*q2 + q3r*q3;
    u_1 = _2_kps_omegas_4*(q1r*q0 - q0r*q1 + q3r*q2 - q2r*q3)/q0rq0_q1rq1_q2rq2_q3rq3 - kds*omega_x - kpw*theta_1 - kdw*omega_1;
    u_2 = _2_kps_omegas_4*(q2r*q0 - q3r*q1 - q0r*q2 + q1r*q3)/q0rq0_q1rq1_q2rq2_q3rq3 - kds*omega_y - kpw*theta_2 - kdw*omega_2;
    u_3 = _2_kps_omegas_4*(q3r*q0 + q2r*q1 - q1r*q2 - q0r*q3)/q0rq0_q1rq1_q2rq2_q3rq3 - kds*omega_z - kpw*theta_3 - kdw*omega_3;
}   

// Feedback linearization step
void AttitudeWheelController3D::feedback_linearization(float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float omega_1, float omega_2, float omega_3)
{
    // Calculate friction torque
    float sign_1, sign_2, sign_3;
    if (omega_1 == 0.0)
    {
        sign_1 = 0.0;
    }
    else
    {
        sign_1 = omega_1/abs(omega_1);
    }
    if (omega_2 == 0.0)
    {
        sign_2 = 0.0;
    }
    else
    {
        sign_2 = omega_2/abs(omega_2);
    }
    if (omega_3 == 0.0)
    {
        sign_3 = 0.0;
    }
    else
    {
        sign_3 = omega_3/abs(omega_3);
    }
    float tau_f_1 = sign_1*(tau_c + b*abs(omega_1) + kd*pow(omega_1,2));
    float tau_f_2 = sign_2*(tau_c + b*abs(omega_2) + kd*pow(omega_2,2));
    float tau_f_3 = sign_3*(tau_c + b*abs(omega_3) + kd*pow(omega_3,2));
    // Feedback linearization step (with auxiliary variables to avoid double arithmetic)     
    float g_l_ms_2_mw = g*l*(m_s + 2.0*m_w);
    tau_1 = /*- I_c_pi*(omega_y - omega_z)*(omega_x + omega_y + omega_z) + g_l_ms_2_mw*(1.0 - q0*q0 + q0*q1 + q2*q3 - q3*q3) +*/ tau_f_1 - I_c*u_1 - I_c_pi*(u_2 + u_3);
    tau_2 = /*- I_c_pi*(omega_z - omega_x)*(omega_x + omega_y + omega_z) + g_l_ms_2_mw*(1.0 + q0*q2 - q1*q1 - q1*q3 - q2*q2) +*/ tau_f_2 - I_c*u_2 - I_c_pi*(u_1 + u_3);
    tau_3 = /*- I_c_pi*(omega_x - omega_y)*(omega_x + omega_y + omega_z) - g_l_ms_2_mw*(      q0*q1 + q0*q2 - q1*q3 + q2*q3) +*/ tau_f_3 - I_c*u_3 - I_c_pi*(u_1 + u_2);
    // Saturation for safety
    if (tau_1 > 0.05)
    {
        tau_1 = 0.05;
    } 
    else if (tau_1 < -0.05)
    {
        tau_1 = -0.05;
    }
    if (tau_2 > 0.05)
    {
        tau_2 = 0.05;
    } 
    else if (tau_2 < -0.05)
    {
        tau_2 = -0.05;
    }
    if (tau_3 > 0.05)
    {
        tau_3 = 0.05;
    } 
    else if (tau_3 < -0.05)
    {
        tau_3 = -0.05;
    }
}         