#include "mbed.h"
#include "attitude_estimator.h"

// Class constructor
AttitudeEstimator::AttitudeEstimator(float frequency) : /*imu(A4,A5)*/ imu(PB_7,PB_6)
{
    dt = 1.0f/frequency;
    dt_2 = dt/2.0f;
}

// Initialize class
void AttitudeEstimator::init()
{
    // Initialize IMU sensor object
    imu.init();
    //
    q = eye(4,1);
    omega = zeros(3,1);
    // Initialize matrices
    x = eye(7,1);
    x(5,1) = 0.008985;
    x(6,1) = -0.078963;
    x(7,1) = 0.040693;
    z = zeros(4,1);
    A = eye(7);
    B = zeros(7,3);
    H = eye(4,7);
    P = eye(7);
    Q = 2.4e-6*eye(3);
    R = 4.2e-4*eye(4);
    K = zeros(7,4);
    // 
    g = zeros(3,1);
    a = zeros(3,1);
    m = zeros(3,1);
    //
    t1 = zeros(3,1);
    t2 = zeros(3,1);
    t3 = zeros(3,1);
    //
    dcm = zeros(3,3);
}

void AttitudeEstimator::estimate()
{
    // Read IMU sensor data
    read();
    // 
    update_measurement();    
    update_jacobian();
    // Kalman filter predict step
    x = f(x);
    P = A*P*transpose(A)+B*Q*transpose(B);
    // Kalman filter correct/update step
    K = P*transpose(H)*inverse(H*P*transpose(H)+R);
    x = x+K*(z-h(x));
    P = P-K*H*P;
    //
    update_estimated_states();
}


void AttitudeEstimator::read()
{
    // Read imu sensor data
    imu.read();
    // Calibrate scale and bias
    g(1,1) = imu.gx;
    g(2,1) = imu.gy;
    g(3,1) = imu.gz;
    a(1,1) = 1.0077f*(imu.ax-0.0975f);
    a(2,1) = 1.0061f*(imu.ay-0.0600f);
    a(3,1) = 0.9926f*(imu.az-0.5156f);
    m(1,1) = 0.8838f*(imu.mx+21.0631f);
    m(2,1) = 1.1537f*(imu.my+8.9233f);
    m(3,1) = 0.9982f*(imu.mz-11.8958f);
}

void AttitudeEstimator::update_measurement()
{
    // Normalize 
    a = a/norm(a);
    m = m/norm(m);
    // 
    t1 = a;
    t2 = cross(a,m);
    t2 = t2/norm(t2);
    t3 = cross(t1,t2);
    // 
    dcm(1,1) = -t3(1,1);
    dcm(2,1) = -t3(2,1);
    dcm(3,1) = -t3(3,1);
    dcm(1,2) = -t2(1,1);
    dcm(2,2) = -t2(2,1);
    dcm(3,2) = -t2(3,1);
    dcm(1,3) = -t1(1,1);
    dcm(2,3) = -t1(2,1);
    dcm(3,3) = -t1(3,1);
    // 
    z = dcm2quat(dcm);
    //
    if((abs(x(1,1)) > abs(x(2,1))) && (abs(x(1,1)) > abs(x(3,1))) && (abs(x(1,1)) > abs(x(4,1)))) {
        if (((x(1,1) > 0) && (z(1,1) < 0)) || ((x(1,1) < 0) && (z(1,1) > 0))) {
            z = -z;
        }
    } else if ((abs(x(2,1)) > abs(x(3,1))) && (abs(x(2,1)) > abs(x(4,1)))) {
        if (((x(2,1) > 0) && (z(2,1) < 0)) || ((x(2,1) < 0) && (z(2,1) > 0))) {
            z = -z;
        }
    } else if ((abs(x(3,1)) > abs(x(4,1)))) {
        if (((x(3,1) > 0) && (z(3,1) < 0)) || ((x(3,1) < 0) && (z(3,1) > 0))) {
            z = -z;
        }
    } else {
        if (((x(4,1) > 0) && (z(4,1) < 0)) || ((x(4,1) < 0) && (z(4,1) > 0))) {
            z = -z;
        }
    }
}

void AttitudeEstimator::update_jacobian()
{
    // Auxiliary variables to avoid double arithmetic
    float omega_x_dt_2 = (g(1,1)-x(5,1))*dt_2;
    float omega_y_dt_2 = (g(2,1)-x(6,1))*dt_2;
    float omega_z_dt_2 = (g(3,1)-x(7,1))*dt_2;
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
Matrix AttitudeEstimator::f(Matrix& x)
{
    // Auxiliary matrix
    Matrix fx(x.rows,x.cols);
    // Auxiliary variables to avoid double arithmetic
    float omega_x = g(1,1)-x(5,1);
    float omega_y = g(2,1)-x(6,1);
    float omega_z = g(3,1)-x(7,1);
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
Matrix AttitudeEstimator::h(Matrix& x)
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

void AttitudeEstimator::update_estimated_states()
{
    q(1,1) = x(1,1);
    q(2,1) = x(2,1);
    q(3,1) = x(3,1);
    q(4,1) = x(4,1);
    omega(1,1) = g(1,1)-x(5,1);
    omega(2,1) = g(2,1)-x(6,1);
    omega(3,1) = g(3,1)-x(7,1);
}