#include "ekf.h"

// Class constructor
ExtendedKalmanFilter::ExtendedKalmanFilter(float freq)
{
    dt = 1.0f/freq;
    dt_2 = dt/2.0f;

    // Initialize matrices
    x = eye(7,1);
    // 
    A = eye(7);
    B = zeros(7,3);
    H = eye(4,7);
    P = eye(7);
    Q = 2.4e-6*eye(3);
    R = 4.2e-4*eye(4);
    K = zeros(7,4);
}

void ExtendedKalmanFilter::predict(Matrix& u)
{
    // 
    update_A(u);
    update_B();
    // Kalman filter prediction step
    x = f(x,u);
    P = A*P*transpose(A)+B*Q*transpose(B);
}

void ExtendedKalmanFilter::correct(Matrix& z)
{
    K = P*transpose(H)*inverse(H*P*transpose(H)+R);
    x = x+K*(z-h(x));
    P = P-K*H*P;
}

void ExtendedKalmanFilter::update_A(Matrix& u)
{
    // Auxiliary variables to avoid double arithmetic
    float omega_x_dt_2 = (u(1,1)-x(5,1))*dt_2;
    float omega_y_dt_2 = (u(2,1)-x(6,1))*dt_2;
    float omega_z_dt_2 = (u(3,1)-x(7,1))*dt_2;
    float q0_dt_2 = x(1,1)*dt_2;
    float q1_dt_2 = x(2,1)*dt_2;
    float q2_dt_2 = x(3,1)*dt_2;
    float q3_dt_2 = x(4,1)*dt_2;
    // Linearized state transition matrix
    A(1,2) = -omega_x_dt_2;
    A(1,3) = -omega_y_dt_2;
    A(1,4) = -omega_z_dt_2;
    A(1,5) = q1_dt_2;
    A(1,6) = q2_dt_2;
    A(1,7) = q3_dt_2;
    A(2,1) = omega_x_dt_2;
    A(2,3) = omega_z_dt_2;
    A(2,4) = -omega_y_dt_2;
    A(2,5) = -q0_dt_2;
    A(2,6) = q3_dt_2;
    A(2,7) = -q2_dt_2;
    A(3,1) = omega_y_dt_2;
    A(3,2) = -omega_z_dt_2;
    A(3,4) = omega_x_dt_2;
    A(3,5) = -q3_dt_2;
    A(3,6) = -q0_dt_2;
    A(3,7) = q1_dt_2;
    A(4,1) = omega_z_dt_2;
    A(4,2) = omega_y_dt_2;
    A(4,3) = -omega_x_dt_2;
    A(4,5) = q2_dt_2;
    A(4,6) = -q1_dt_2;
    A(4,7) = -q0_dt_2;
}

void ExtendedKalmanFilter::update_B()
{
    // Auxiliary variables to avoid double arithmetic
    float q0_dt_2 = x(1,1)*dt_2;
    float q1_dt_2 = x(2,1)*dt_2;
    float q2_dt_2 = x(3,1)*dt_2;
    float q3_dt_2 = x(4,1)*dt_2;
    // Linearized input matrix
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
}

// State transition function x = f(x)
Matrix ExtendedKalmanFilter::f(Matrix& x, Matrix& u)
{
    // Auxiliary matrix
    Matrix fx(x.rows,x.cols);
    // Auxiliary variables to avoid double arithmetic
    float omega_x = u(1,1)-x(5,1);
    float omega_y = u(2,1)-x(6,1);
    float omega_z = u(3,1)-x(7,1);
    // State transition function
    fx(1,1) = x(1,1)+(-x(2,1)*omega_x-x(3,1)*omega_y-x(4,1)*omega_z)*dt_2;
    fx(2,1) = x(2,1)+(x(1,1)*omega_x-x(4,1)*omega_y+x(3,1)*omega_z)*dt_2;
    fx(3,1) = x(3,1)+(x(4,1)*omega_x+x(1,1)*omega_y-x(2,1)*omega_z)*dt_2;
    fx(4,1) = x(4,1)+(-x(3,1)*omega_x+x(2,1)*omega_y+x(1,1)*omega_z)*dt_2;
    fx(5,1) = x(5,1);
    fx(6,1) = x(6,1);
    fx(7,1) = x(7,1);
    // Normalize quaternion with axuliary variable to avoid double arithmetic
    float q_norm = 1.0f/sqrt(fx(1,1)*fx(1,1)+fx(2,1)*fx(2,1)+fx(3,1)*fx(3,1)+fx(4,1)*fx(4,1));
    fx(1,1) = fx(1,1)*q_norm;
    fx(2,1) = fx(2,1)*q_norm;
    fx(3,1) = fx(3,1)*q_norm;
    fx(4,1) = fx(4,1)*q_norm;
    // Return f(x)
    return fx;
}

// Output function z = h(x)
Matrix ExtendedKalmanFilter::h(Matrix& x)
{
    // Auxiliary variable
    Matrix hx(4,1);
    // Output function
    hx(1,1) = x(1,1);
    hx(2,1) = x(2,1);
    hx(3,1) = x(3,1);
    hx(4,1) = x(4,1);
    // Return h(x)
    return hx;
}