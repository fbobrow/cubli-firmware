#ifndef attitude_wheel_controller_3d_h
#define attitude_wheel_controller_3d_h

#include "mbed.h"

#include "src/utils/parameters.h"

// Speed estimator class
class AttitudeWheelController3D
{
  public:
    // Constructor
    AttitudeWheelController3D();
    // Control step
    void control(float q0r, float q1r, float q2r, float q3r, float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float theta_1, float theta_2, float theta_3, float omega_1, float omega_2, float omega_3);
    // Torque [N.m]
    float tau_1, tau_2, tau_3;
  private:
    // State regulator step
    void state_regulator(float q0r, float q1r, float q2r, float q3r, float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float theta_1, float theta_2, float theta_3, float omega_1, float omega_2, float omega_3);
    // Feedback linearization step
    void feedback_linearization(float q0, float q1, float q2, float q3, float omega_x, float omega_y, float omega_z, float omega_1, float omega_2, float omega_3);
    // Linearized input
    float u_1, u_2, u_3;

};

#endif