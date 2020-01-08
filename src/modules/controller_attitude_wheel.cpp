#include "controller_attitude_wheel.h"

// Constructor
AttitudeWheelController::AttitudeWheelController()
{
    // Set initial quaternion error
    qe0 = 0.0;
    qe1 = 0.0;
    qe2 = 0.0;
    qe3 = 0.0;
    // Set initial torque 
    tau_1 = 0.0;
    tau_2 = 0.0;
    tau_3 = 0.0;
    // Set initial linearized input
    u_1 = 0.0;
    u_2 = 0.0;
    u_3 = 0.0;
}

// Control step
void AttitudeWheelController::control(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float theta_1, float theta_2, float theta_3, float omega_1, float omega_2, float omega_3)
{
    state_regulator(qr0,qr1,qr2,qr3,q0,q1,q2,q3,omega_x,omega_y,omega_z,theta_1,theta_2,theta_3,omega_1,omega_2,omega_3);
    feedback_linearization(q0,q1,q2,q3,omega_x,omega_y,omega_z,omega_1,omega_2,omega_3);
}   

// State regulator step
void AttitudeWheelController::state_regulator(float qr0, float qr1, float qr2, float qr3, float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float theta_1, float theta_2, float theta_3, float omega_1, float omega_2, float omega_3)
{
    // Calculate quaternion error
    qe0 = q0*qr0 + q1*qr1 + q2*qr2 + q3*qr3;
    qe1 = q0*qr1 - q1*qr0 - q2*qr3 + q3*qr2;
    qe2 = q0*qr2 - q2*qr0 + q1*qr3 - q3*qr1;
    qe3 = q0*qr3 - q1*qr2 + q2*qr1 - q3*qr0;
    // Normalize quaternion error
    float qe_norm = sqrt(qe0*qe0+qe1*qe1+qe2*qe2+qe3*qe3);
    qe0 /= qe_norm;
    qe1 /= qe_norm;
    qe2 /= qe_norm;
    qe3 /= qe_norm;  
    // State regulator step (with auxiliary variables to avoid double arithmetic) 
    float _2_kps_omegas_4 = 2.0*(kps - (omega_x*omega_x + omega_y*omega_y + omega_z*omega_z)/4.0);
    u_1 = _2_kps_omegas_4*(qe1)/qe0 - kds*omega_x - kpw*theta_1 - kdw*omega_1;
    u_2 = _2_kps_omegas_4*(qe2)/qe0 - kds*omega_y - kpw*theta_2 - kdw*omega_2;
    u_3 = _2_kps_omegas_4*(qe3)/qe0 - kds*omega_z - kpw*theta_3 - kdw*omega_3;
    
}  

// Feedback linearization step
void AttitudeWheelController::feedback_linearization(float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float omega_1, float omega_2, float omega_3)
{
    // Calculate friction torque
    float sign_1 = (0.0<omega_1)-(omega_1<0.0); 
    float sign_2 = (0.0<omega_2)-(omega_2<0.0); 
    float sign_3 = (0.0<omega_3)-(omega_3<0.0); 
    float tau_f_1 = sign_1*(tau_c + b*abs(omega_1) + kd*omega_1*omega_1);
    float tau_f_2 = sign_2*(tau_c + b*abs(omega_2) + kd*omega_2*omega_2);
    float tau_f_3 = sign_3*(tau_c + b*abs(omega_3) + kd*omega_3*omega_3);
    // Feedback linearization step (with auxiliary variables to avoid double arithmetic)
    tau_1 = - I_c_p*(omega_y - omega_z)*(omega_x + omega_y + omega_z) + g_l_ms_2_mw*(0.5 - q0*q0 + q0*q1 - q3*q3 + q2*q3) + tau_f_1 - I_c*u_1 - I_c_p*(u_2 + u_3);
    tau_2 = - I_c_p*(omega_z - omega_x)*(omega_x + omega_y + omega_z) + g_l_ms_2_mw*(0.5 + q0*q2 - q1*q1 - q1*q3 - q2*q2) + tau_f_2 - I_c*u_2 - I_c_p*(u_1 + u_3);
    tau_3 = - I_c_p*(omega_x - omega_y)*(omega_x + omega_y + omega_z) - g_l_ms_2_mw*(      q0*q1 + q0*q2 - q1*q3 + q2*q3) + tau_f_3 - I_c*u_3 - I_c_p*(u_1 + u_2);  
}         