#include "ekf.h"

// Class constructor
ExtendedKalmanFilter::ExtendedKalmanFilter(float freq, float u_var, float z_var)
{
    // Initialize state vector x and error covariance matrix P
    x = eye(7,1);
    P = eye(7);
    // Initialize state noise covariance matrix Q and measurement noise covariance matrix R
    Q = u_var*eye(3);
    R = z_var*eye(4);
    // Initialize time interval dt and dt/2 (to avoid double arithmetic)
    dt = 1.0f/freq;
    dt_2 = dt/2.0f;
}

// Prediction step
void ExtendedKalmanFilter::predict(const Matrix& u)
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
void ExtendedKalmanFilter::correct(const Matrix& z)
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
Matrix ExtendedKalmanFilter::f(const Matrix& x0, const Matrix& u0)
{
    // Declare f(x,u)
    Matrix f(7,1);
    // Auxiliary variables to avoid double arithmetic
    float omega_x = u0(1,1)-x0(5,1);
    float omega_y = u0(2,1)-x0(6,1);
    float omega_z = u0(3,1)-x0(7,1);
    // Calculate f(x,u)
    f(1,1) = (1.0f/2.0f)*(-x(2,1)*omega_x-x(3,1)*omega_y-x(4,1)*omega_z);
    f(2,1) = (1.0f/2.0f)*(x(1,1)*omega_x-x(4,1)*omega_y+x(3,1)*omega_z);
    f(3,1) = (1.0f/2.0f)*(x(4,1)*omega_x+x(1,1)*omega_y-x(2,1)*omega_z);
    f(4,1) = (1.0f/2.0f)*(-x(3,1)*omega_x+x(2,1)*omega_y+x(1,1)*omega_z);
    // Return f(x,u)
    return f;
}

// Measurement function z = h(x)
Matrix ExtendedKalmanFilter::h(const Matrix& x0)
{
    // Declare h(x)
    Matrix h(4,1);
    // Calculate h(x)
    h(1,1) = x0(1,1);
    h(2,1) = x0(2,1);
    h(3,1) = x0(3,1);
    h(4,1) = x0(4,1);
    // Return h(x)
    return h;
}

// State transition matrix A = jacob(f,x)
Matrix ExtendedKalmanFilter::jacob_f_x(const Matrix& x0, const Matrix& u0)
{
    // Declare A
    Matrix A(7,7);
    // Auxiliary variables to avoid double arithmetic
    float omega_x_dt_2 = (u0(1,1)-x0(5,1))*dt_2;
    float omega_y_dt_2 = (u0(2,1)-x0(6,1))*dt_2;
    float omega_z_dt_2 = (u0(3,1)-x0(7,1))*dt_2;
    float q0_dt_2 = x0(1,1)*dt_2;
    float q1_dt_2 = x0(2,1)*dt_2;
    float q2_dt_2 = x0(3,1)*dt_2;
    float q3_dt_2 = x0(4,1)*dt_2;
    // Calculate A
    A(1,1) = 1.0f;
    A(1,2) = -omega_x_dt_2;
    A(1,3) = -omega_y_dt_2;
    A(1,4) = -omega_z_dt_2;
    A(1,5) = q1_dt_2;
    A(1,6) = q2_dt_2;
    A(1,7) = q3_dt_2;
    A(2,1) = omega_x_dt_2;
    A(2,2) = 1.0f;
    A(2,3) = omega_z_dt_2;
    A(2,4) = -omega_y_dt_2;
    A(2,5) = -q0_dt_2;
    A(2,6) = q3_dt_2;
    A(2,7) = -q2_dt_2;
    A(3,1) = omega_y_dt_2;
    A(3,2) = -omega_z_dt_2;
    A(3,3) = 1.0f;
    A(3,4) = omega_x_dt_2;
    A(3,5) = -q3_dt_2;
    A(3,6) = -q0_dt_2;
    A(3,7) = q1_dt_2;
    A(4,1) = omega_z_dt_2;
    A(4,2) = omega_y_dt_2;
    A(4,3) = -omega_x_dt_2;
    A(4,4) = 1.0f;
    A(4,5) = q2_dt_2;
    A(4,6) = -q1_dt_2;
    A(4,7) = -q0_dt_2;
    A(5,5) = 1.0f;
    A(6,6) = 1.0f;
    A(7,7) = 1.0f;
    // Return A
    return A;
}

// Input matrix B = jacob(f,u)
Matrix ExtendedKalmanFilter::jacob_f_u(const Matrix& x0, const Matrix& u0)
{
    // Declare B
    Matrix B(7,3);
    // Auxiliary variables to avoid double arithmetic
    float q0_dt_2 = x0(1,1)*dt_2;
    float q1_dt_2 = x0(2,1)*dt_2;
    float q2_dt_2 = x0(3,1)*dt_2;
    float q3_dt_2 = x0(4,1)*dt_2;
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
Matrix ExtendedKalmanFilter::jacob_h_x(const Matrix& x0)
{
    // Declare H
    Matrix H(4,7);
    // Calculate H
    H(1,1) = 1.0f;
    H(2,2) = 1.0f;
    H(3,3) = 1.0f;
    H(4,4) = 1.0f;
    // Return H
    return H;
}

// Normalize quaternion state q = q/norm(q)
void ExtendedKalmanFilter::norm_quat()
{
    // Auxiliary variable to avoid double arithmetic
    float q_norm = 1.0f/sqrt(x(1,1)*x(1,1)+x(2,1)*x(2,1)+x(3,1)*x(3,1)+x(4,1)*x(4,1));
    // Normalize quaternion state
    x(1,1) = x(1,1)*q_norm;
    x(2,1) = x(2,1)*q_norm;
    x(3,1) = x(3,1)*q_norm;
    x(4,1) = x(4,1)*q_norm;
}