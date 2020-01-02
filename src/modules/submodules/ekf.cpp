#include "ekf.h"

// Class constructor
EKF::EKF()
{
    // Initialize state vector x and error covariance matrix P
    x = eye(7,1);
    x(5,1) = b_gx;
    x(6,1) = b_gy;
    x(7,1) = b_gz;
    P = 1e-8*eye(7);
    P(5,5) = 1e-10;
    P(6,6) = 1e-10;
    P(7,7) = 1e-10;
    // Initialize state noise covariance matrix Q and measurement noise covariance matrix R
    Q = g_cov*eye(3);
    R = a_cov*eye(3);
}

// Prediction step
void EKF::predict(const Matrix& u)
{
    // Compute state transition matrix A = jacob(f,x) and input matrix B = jacob(f,u)
    Matrix A = jacob_f_x(x,u);
    Matrix B = jacob_f_u(x,u);
    // Predict state vector x and error covariance matrix P
    x = x+f(x,u)*dt;
    P = A*P*transpose(A)+B*Q*transpose(B);
    // Normalize quaternion
    norm_quat();
}

// Correction step
void EKF::correct(const Matrix& z)
{
    // Compute measurement matrix H = jacob(h,x)
    Matrix H = jacob_h_x(x);
    // Compute Kalman gain K
    Matrix K = P*transpose(H)*inverse(H*P*transpose(H)+R);
    // Correct state vector x and error covariance matrix P
    x = x+K*(z-h(x));
    P = P-K*H*P;
    // Normalize quaternion
    norm_quat();
}

// State transition function x_dot = f(x,u)
Matrix EKF::f(const Matrix& x, const Matrix& u)
{
    // Auxiliary variables
    float q0 = x(1,1);
    float q1 = x(2,1);
    float q2 = x(3,1);
    float q3 = x(4,1);
    float omega_x = u(1,1)-x(5,1);
    float omega_y = u(2,1)-x(6,1);
    float omega_z = u(3,1)-x(7,1);
    // Declare f(x,u)
    Matrix f(7,1);
    // Calculate f(x,u)
    f(1,1) = (-q1*omega_x-q2*omega_y-q3*omega_z)/2.0;
    f(2,1) = (q0*omega_x-q3*omega_y+q2*omega_z)/2.0;
    f(3,1) = (q3*omega_x+q0*omega_y-q1*omega_z)/2.0;
    f(4,1) = (-q2*omega_x+q1*omega_y+q0*omega_z)/2.0;
    // Return f(x,u)
    return f;
}

// Measurement function z = h(x)
Matrix EKF::h(const Matrix& x)
{
    // Auxiliary variables
    float q0 = x(1,1);
    float q1 = x(2,1);
    float q2 = x(3,1);
    float q3 = x(4,1);
    // Declare h(x) 
    Matrix h(3,1);
    // Calculate h(x)
    h(1,1) = 2.0*g*(q0*q2-q1*q3);
    h(2,1) = -2.0*g*(q0*q1+q2*q3);
    h(3,1) = -g*(q0*q0-q1*q1-q2*q2+q3*q3);
    // Return h(x)
    return h;
}

// State transition matrix A = jacob(f,x)
Matrix EKF::jacob_f_x(const Matrix& x, const Matrix& u)
{
    // Auxiliary variables 
    float q0_dt_2 = x(1,1)*dt_2;
    float q1_dt_2 = x(2,1)*dt_2;
    float q2_dt_2 = x(3,1)*dt_2;
    float q3_dt_2 = x(4,1)*dt_2;
    float omega_x_dt_2 = (u(1,1)-x(5,1))*dt_2;
    float omega_y_dt_2 = (u(2,1)-x(6,1))*dt_2;
    float omega_z_dt_2 = (u(3,1)-x(7,1))*dt_2;
    // Declare A
    Matrix A(7,7);
    // Calculate A
    A(1,1) = 1.0;
    A(1,2) = -omega_x_dt_2;
    A(1,3) = -omega_y_dt_2;
    A(1,4) = -omega_z_dt_2;
    A(1,5) = q1_dt_2;
    A(1,6) = q2_dt_2;
    A(1,7) = q3_dt_2;
    A(2,1) = omega_x_dt_2;
    A(2,2) = 1.0;
    A(2,3) = omega_z_dt_2;
    A(2,4) = -omega_y_dt_2;
    A(2,5) = -q0_dt_2;
    A(2,6) = q3_dt_2;
    A(2,7) = -q2_dt_2;
    A(3,1) = omega_y_dt_2;
    A(3,2) = -omega_z_dt_2;
    A(3,3) = 1.0;
    A(3,4) = omega_x_dt_2;
    A(3,5) = -q3_dt_2;
    A(3,6) = -q0_dt_2;
    A(3,7) = q1_dt_2;
    A(4,1) = omega_z_dt_2;
    A(4,2) = omega_y_dt_2;
    A(4,3) = -omega_x_dt_2;
    A(4,4) = 1.0;
    A(4,5) = q2_dt_2;
    A(4,6) = -q1_dt_2;
    A(4,7) = -q0_dt_2;
    A(5,5) = 1.0;
    A(6,6) = 1.0;
    A(7,7) = 1.0;
    // Return A
    return A;
}

// Input matrix B = jacob(f,u)
Matrix EKF::jacob_f_u(const Matrix& x, const Matrix& u)
{
    // Auxiliary variables 
    float q0_dt_2 = x(1,1)*dt_2;
    float q1_dt_2 = x(2,1)*dt_2;
    float q2_dt_2 = x(3,1)*dt_2;
    float q3_dt_2 = x(4,1)*dt_2;
    // Declare B 
    Matrix B(7,3);
    // Calculate B
    B(1,1) = -q1_dt_2;
    B(1,2) = -q2_dt_2;
    B(1,3) = -q3_dt_2;
    B(2,1) = q0_dt_2;
    B(2,2) = -q3_dt_2;
    B(2,3) = q2_dt_2;
    B(3,1) = q3_dt_2;
    B(3,2) = q0_dt_2;
    B(3,3) = -q1_dt_2;
    B(4,1) = -q2_dt_2;
    B(4,2) = q1_dt_2;
    B(4,3) = q0_dt_2; 
    // Return B
    return B;
}

// Measurement matrix H = jacob(h,x)
Matrix EKF::jacob_h_x(const Matrix& x)
{
    // Auxiliary variables
    float q0_g_2 = x(1,1)*g*2.0;
    float q1_g_2 = x(2,1)*g*2.0;
    float q2_g_2 = x(3,1)*g*2.0;
    float q3_g_2 = x(4,1)*g*2.0;
    // Declare H
    Matrix H(3,7);
    // Calculate H
    H(1,1) = q2_g_2;
    H(1,2) = -q3_g_2;
    H(1,3) = q0_g_2;
    H(1,4) = -q1_g_2;
    H(2,1) = -q1_g_2;
    H(2,2) = -q0_g_2;
    H(2,3) = -q3_g_2;
    H(2,4) = -q2_g_2;
    H(3,1) = -q0_g_2;
    H(3,2) = q1_g_2;
    H(3,3) = q2_g_2;
    H(3,4) = -q3_g_2;
    // Return H
    return H;
}

// Normalize quaternion state q = q/norm(q)
void EKF::norm_quat()
{
    // Auxiliary variable to avoid double arithmetic
    float q_norm = 1.0/sqrt(x(1,1)*x(1,1)+x(2,1)*x(2,1)+x(3,1)*x(3,1)+x(4,1)*x(4,1));
    // Normalize quaternion state
    x(1,1) = x(1,1)*q_norm;
    x(2,1) = x(2,1)*q_norm;
    x(3,1) = x(3,1)*q_norm;
    x(4,1) = x(4,1)*q_norm;
}