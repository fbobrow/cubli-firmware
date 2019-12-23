#include "triad.h"

// Class constructor
Triad::Triad()
{
    // Initialize orientation quaternion
    q = eye(4,1);
    // Initialize direct cossine matrix from inertial reference frame to triad reference frame
    Matrix u_I(3,1), v_I(3,1);
    u_I(1,1) = ax_I;
    u_I(2,1) = ay_I;
    u_I(3,1) = az_I;
    v_I(1,1) = mx_I;
    v_I(2,1) = my_I;
    v_I(3,1) = mz_I;
    Matrix R_IT = triad(u_I,v_I);
    R_TI = transpose(R_IT);
}

// Update step
void Triad::update(const Matrix &u_B, const Matrix &v_B)
{
    // Calculate direct cossine matrix from inertial reference frame to body fixed reference frame
    Matrix R_BT = triad(u_B,v_B);
    Matrix R_BI = R_BT*R_TI;
    // Convert direct cossine matrix to quaternions
    q = dcm2quat(R_BI);
}

// Triad method
Matrix Triad::triad(const Matrix& u, const Matrix &v)
{
    // Calculate triad vectors
    Matrix t1 = u/norm(u);
    Matrix t2 = cross(u,v)/norm(cross(u,v));
    Matrix t3 = cross(t1,t2);
    // Calculate direct cossine matrix from triad vectors
    Matrix R(3,3);
    R(1,1) = t1(1,1);
    R(2,1) = t1(2,1);
    R(3,1) = t1(3,1);
    R(1,2) = t2(1,1);
    R(2,2) = t2(2,1);
    R(3,2) = t2(3,1);
    R(1,3) = t3(1,1);
    R(2,3) = t3(2,1);
    R(3,3) = t3(3,1);
    // Return direct cossine matrix
    return R;
}

// Convert direct cossine matrix to quaternions
Matrix Triad::dcm2quat(const Matrix& R)
{
    Matrix q(4,1);
    float t = trace(R);
    if (t > 0.0) {
        float sqtrp1 = sqrt(t+1.0);
        q(1,1) = 0.5*sqtrp1;
        q(2,1) = (R(2,3)-R(3,2))/(2.0*sqtrp1);
        q(3,1) = (R(3,1)-R(1,3))/(2.0*sqtrp1);
        q(4,1) = (R(1,2)-R(2,1))/(2.0*sqtrp1);
    } else {
        if ((R(2,2) > R(1,1)) && (R(2,2) > R(3,3))) {
            float sqdip1 = sqrt(R(2,2)-R(1,1)-R(3,3)+1.0);
            q(3,1) = 0.5*sqdip1;
            if (sqdip1 != 0.0) {
                sqdip1 = 0.5/sqdip1;
            }
            q(1,1) = (R(3,1)-R(1,3))*sqdip1;
            q(2,1) = (R(1,2)+R(2,1))*sqdip1;
            q(4,1) = (R(2,3)+R(3,2))*sqdip1;
        } else if (R(3,3) > R(1,1)) {
            float sqdip1 = sqrt(R(3,3)-R(1,1)-R(2,2)+1.0);
            q(4,1) = 0.5*sqdip1;
            if (sqdip1 != 0.0) {
                sqdip1 = 0.5/sqdip1;
            }
            q(1,1) = (R(1,2)-R(2,1))*sqdip1;
            q(2,1) = (R(3,1)+R(1,3))*sqdip1;
            q(3,1) = (R(2,3)+R(3,2))*sqdip1;
        } else {
            float sqdip1 = sqrt(R(1,1)-R(2,2)-R(3,3)+1.0);
            q(2,1) = 0.5*sqdip1;
            if (sqdip1 != 0.0) {
                sqdip1 = 0.5/sqdip1;
            }
            q(1,1) = (R(2,3)-R(3,2))*sqdip1;
            q(3,1) = (R(1,2)+R(2,1))*sqdip1;
            q(4,1) = (R(3,1)+R(1,3))*sqdip1;
        }
    }
    return q;
}