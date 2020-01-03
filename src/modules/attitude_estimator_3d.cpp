#include "attitude_estimator_3d.h"

// Constructor
AttitudeEstimator3D::AttitudeEstimator3D(PinName PIN_SDA, PinName PIN_SCL) : imu(PIN_SDA,PIN_SCL)
{
    // Initialize quaternion and angular velocity vector
    x = eye(4,1);
    //
    P = 1.0e-8*eye(4);
    // Initialize state noise covariance matrix Q and measurement noise covariance matrix R
    Q = g_cov*eye(3);
    R = a_cov*eye(3);
}

// Class initializer
void AttitudeEstimator3D::init()
{
    // Initialize IMU object
    imu.init();
    calibrate();
}


// Angular velocity bias calibration 
void AttitudeEstimator3D::calibrate()
{
    // Calculate angular velocity bias by averaging 200 samples durint 1 second
    for(int i = 0; i<200;i++)
    {
        imu.read();
        b_omega_x += imu.gx/200.0;
        b_omega_y += imu.gy/200.0;
        b_omega_z += imu.gz/200.0;
        wait_us(dt_us);
    }
}

// Update step
void AttitudeEstimator3D::estimate()
{
    // Read IMU data
    imu.read();
    // Assignf gyroscope, accelerometer and magnetometer data to vectors 
    Matrix gyr(3,1), acc(3,1);
    gyr(1,1) = imu.gx-b_omega_x;
    gyr(2,1) = imu.gy-b_omega_y;
    gyr(3,1) = imu.gz-b_omega_z;
    acc(1,1) = f_ax*(imu.ax-b_ax);
    acc(2,1) = f_ay*(imu.ay-b_ay);
    acc(3,1) = f_az*(imu.az-b_az);
    acc = acc/norm(acc);
    // Extended Kalman filter predict step
    predict(gyr);
    // Extended Kalman filter correction step
    correct(acc);
    // Update orientation quaternion and angular velocity vector
    q0 = x(1,1);
    q1 = x(2,1);
    q2 = x(3,1);
    q3 = x(4,1);
    omega_x = gyr(1,1);
    omega_y = gyr(2,1);
    omega_z = gyr(3,1);
}



// Prediction step
void AttitudeEstimator3D::predict(const Matrix& u)
{
    // Compute state transition matrix A = jacob(f,x) and input matrix B = jacob(f,u)
    Matrix A = jacob_f_x(x,u);
    Matrix B = jacob_f_u(x,u);
    // Predict state vector x and error covariance matrix P
    x = x+f(x,u)*dt;
    P = A*P*transpose(A)+B*Q*transpose(B);
    // Normalize quaternion
    x = x/norm(x);
}

// Correction step
void AttitudeEstimator3D::correct(const Matrix& z)
{
    // Compute measurement matrix H = jacob(h,x)
    Matrix H = jacob_h_x(x);
    // Compute Kalman gain K
    Matrix K = P*transpose(H)*inverse(H*P*transpose(H)+R);
    // Correct state vector x and error covariance matrix P
    x = x+K*(z-h(x));
    P = P-K*H*P;
    // Normalize quaternion
    x = x/norm(x);
}

// State transition function x_dot = f(x,u)
Matrix AttitudeEstimator3D::f(const Matrix& x, const Matrix& u)
{
    // Auxiliary variables
    float _q0 = x(1,1);
    float _q1 = x(2,1);
    float _q2 = x(3,1);
    float _q3 = x(4,1);
    float _omega_x = u(1,1);
    float _omega_y = u(2,1);
    float _omega_z = u(3,1);
    // Declare f(x,u)
    Matrix f(7,1);
    // Calculate f(x,u)
    f(1,1) = 0.5*(-_q1*_omega_x-_q2*_omega_y-_q3*_omega_z);
    f(2,1) = 0.5*(_q0*_omega_x-_q3*_omega_y+_q2*_omega_z);
    f(3,1) = 0.5*(_q3*_omega_x+_q0*_omega_y-_q1*_omega_z);
    f(4,1) = 0.5*(-_q2*_omega_x+_q1*_omega_y+_q0*_omega_z);
    // Return f(x,u)
    return f;
}

// Measurement function z = h(x)
Matrix AttitudeEstimator3D::h(const Matrix& x)
{
    // Auxiliary variables
    float _q0 = x(1,1);
    float _q1 = x(2,1);
    float _q2 = x(3,1);
    float _q3 = x(4,1);
    // Declare h(x) 
    Matrix h(3,1);
    // Calculate h(x)
    h(1,1) = 2.0*(_q0*_q2-_q1*_q3);
    h(2,1) = -2.0*(_q0*_q1+_q2*_q3);
    h(3,1) = -(_q0*_q0-_q1*_q1-_q2*_q2+_q3*_q3);
    // Return h(x)
    return h;
}

// State transition matrix A = jacob(f,x)
Matrix AttitudeEstimator3D::jacob_f_x(const Matrix& x, const Matrix& u)
{
    // Auxiliary variables 
    float omega_x_dt_2 = u(1,1)*dt_2;
    float omega_y_dt_2 = u(2,1)*dt_2;
    float omega_z_dt_2 = u(3,1)*dt_2;
    // Declare A
    Matrix A(4,4);
    // Calculate A
    A(1,1) = 1.0;
    A(1,2) = -omega_x_dt_2;
    A(1,3) = -omega_y_dt_2;
    A(1,4) = -omega_z_dt_2;
    A(2,1) = omega_x_dt_2;
    A(2,2) = 1.0;
    A(2,3) = omega_z_dt_2;
    A(2,4) = -omega_y_dt_2;
    A(3,1) = omega_y_dt_2;
    A(3,2) = -omega_z_dt_2;
    A(3,3) = 1.0;
    A(3,4) = omega_x_dt_2;
    A(4,1) = omega_z_dt_2;
    A(4,2) = omega_y_dt_2;
    A(4,3) = -omega_x_dt_2;
    A(4,4) = 1.0;
    // Return A
    return A;
}

// Input matrix B = jacob(f,u)
Matrix AttitudeEstimator3D::jacob_f_u(const Matrix& x, const Matrix& u)
{
    // Auxiliary variables 
    float q0_dt_2 = x(1,1)*dt_2;
    float q1_dt_2 = x(2,1)*dt_2;
    float q2_dt_2 = x(3,1)*dt_2;
    float q3_dt_2 = x(4,1)*dt_2;
    // Declare B 
    Matrix B(4,3);
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
Matrix AttitudeEstimator3D::jacob_h_x(const Matrix& x)
{
    // Auxiliary variables
    float _2_q0 = 2.0*x(1,1);
    float _2_q1 = 2.0*x(2,1);
    float _2_q2 = 2.0*x(3,1);
    float _2_q3 = 2.0*x(4,1);
    // Declare H
    Matrix H(3,4);
    // Calculate H
    H(1,1) = _2_q2;
    H(1,2) = -_2_q3;
    H(1,3) = _2_q0;
    H(1,4) = -_2_q1;
    H(2,1) = -_2_q1;
    H(2,2) = -_2_q0;
    H(2,3) = -_2_q3;
    H(2,4) = -_2_q2;
    H(3,1) = -_2_q0;
    H(3,2) = _2_q1;
    H(3,3) = _2_q2;
    H(3,4) = -_2_q3;
    // Return H
    return H;
}
            