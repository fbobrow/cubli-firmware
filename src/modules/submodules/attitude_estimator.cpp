#include "attitude_estimator.h"

// Constructor
AttitudeEstimator::AttitudeEstimator() : imu(IMU_SDA,IMU_SCL)
{
    theta_s = 0.0;
    omega_s = 0.0;
}

// Initializer
void AttitudeEstimator::init()
{
    imu.init();
    calibrate();
}

// Angular velocity bias calibration 
void AttitudeEstimator::calibrate()
{
    for(int i = 0; i<200;i++)
    {
        imu.read();
        b_omega_s += imu.gx/200.0;
        wait_us(dt_us);
    }
}

// Predict step
void AttitudeEstimator::estimate()
{
    imu.read();
    omega_s_m = imu.gx-b_omega_s;
    theta_s_m = atan2(-imu.ay,-imu.az);

    omega_s = omega_s_m;
    theta_s = theta_s+omega_s*dt;

    theta_s = theta_s+1*dt*(theta_s_m-theta_s);

}
            