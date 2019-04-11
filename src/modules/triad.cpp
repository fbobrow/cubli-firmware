#include "triad.h"

//
Triad::Triad()
{
    //
    q = eye(4,1);
    Matrix a_inertial(3,1), m_inertial(3,1);
    a_inertial(1,1) = 0.0f;
    a_inertial(2,1) = 0.0f;
    a_inertial(3,1) = 9.81f;
    m_inertial(1,1) = 16.730f;
    m_inertial(2,1) = -6.598f;
    m_inertial(3,1) = -14.140f;
    NT = triad(a_inertial,m_inertial);
    NT_T = transpose(NT);
}

void Triad::update(const Matrix &u, const Matrix &v)
{
    Matrix BT = triad(u,v);
    Matrix BN = BT*NT_T;
    q = dcm2quat(BN);
}

//
Matrix Triad::triad(const Matrix& u, const Matrix &v)
{
    // 
    Matrix R(3,3);
    // 
    Matrix t1 = u/norm(u);
    Matrix t2 = cross(t1,v/norm(v));
    t2 = t2/norm(t2);
    Matrix t3 = cross(t1,t2);
    //
    R(1,1) = t1(1,1);
    R(2,1) = t1(2,1);
    R(3,1) = t1(3,1);
    R(1,2) = t2(1,1);
    R(2,2) = t2(2,1);
    R(3,2) = t2(3,1);
    R(1,3) = t3(1,1);
    R(2,3) = t3(2,1);
    R(3,3) = t3(3,1);
    // 
    return R;
}

//
Matrix Triad::dcm2quat(const Matrix& R)
{
    Matrix q(4,1);
    float t = trace(R);
    if (t > 0.0f) {
        float sqtrp1 = sqrt(t+1.0f);
        q(1,1) = 0.5f*sqtrp1;
        q(2,1) = (R(2,3)-R(3,2))/(2.0f*sqtrp1);
        q(3,1) = (R(3,1)-R(1,3))/(2.0f*sqtrp1);
        q(4,1) = (R(1,2)-R(2,1))/(2.0f*sqtrp1);
    } else {
        if ((R(2,2) > R(1,1)) && (R(2,2) > R(3,3))) {
            float sqdip1 = sqrt(R(2,2)-R(1,1)-R(3,3)+1.0f);
            q(3,1) = 0.5f*sqdip1;
            if (sqdip1 != 0.0f) {
                sqdip1 = 0.5f/sqdip1;
            }
            q(1,1) = (R(3,1)-R(1,3))*sqdip1;
            q(2,1) = (R(1,2)+R(2,1))*sqdip1;
            q(4,1) = (R(2,3)+R(3,2))*sqdip1;
        } else if (R(3,3) > R(1,1)) {
            float sqdip1 = sqrt(R(3,3)-R(1,1)-R(2,2)+1.0f);
            q(4,1) = 0.5f*sqdip1;
            if (sqdip1 != 0.0f) {
                sqdip1 = 0.5f/sqdip1;
            }
            q(1,1) = (R(1,2)-R(2,1))*sqdip1;
            q(2,1) = (R(3,1)+R(1,3))*sqdip1;
            q(3,1) = (R(2,3)+R(3,2))*sqdip1;
        } else {
            float sqdip1 = sqrt(R(1,1)-R(2,2)-R(3,3)+1.0f);
            q(2,1) = 0.5f*sqdip1;
            if (sqdip1 != 0.0f) {
                sqdip1 = 0.5f/sqdip1;
            }
            q(1,1) = (R(2,3)-R(3,2))*sqdip1;
            q(3,1) = (R(1,2)+R(2,1))*sqdip1;
            q(4,1) = (R(3,1)+R(1,3))*sqdip1;
        }
    }
    return q;
}