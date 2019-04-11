#include "estimator.h"


Estimator::Estimator(float freq) : ekf(freq,2.4e-6f,4.2e-4f)
{
    //
    dt = 1.0f/freq;
    dt_2 = dt/2.0f;
    //
    q = eye(4,1);
    omega = zeros(3,1);
}

void Estimator::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    //
    Matrix g(3,1), a(3,1), m(3,1), z(4,1);
    //
    g(1,1) = gx;
    g(2,1) = gy;
    g(3,1) = gz;
    //
    ekf.predict(g);
    //
    a(1,1) = f_ax*(ax-b_ax);
    a(2,1) = f_ay*(ay-b_ay);
    a(3,1) = f_az*(az-b_az);
    m(1,1) = f_mx*(mx-b_mx);
    m(2,1) = f_my*(my-b_my);
    m(3,1) = f_mz*(mz-b_mz);
    //
    //z = triad(a,m); 
    triad.update(a,m);
    z = triad.q;
    // 
    if((abs(ekf.x(1,1)) > abs(ekf.x(2,1))) && (abs(ekf.x(1,1)) > abs(ekf.x(3,1))) && (abs(ekf.x(1,1)) > abs(ekf.x(4,1)))) {
        if (((ekf.x(1,1) > 0) && (z(1,1) < 0)) || ((ekf.x(1,1) < 0) && (z(1,1) > 0))) {
            z = -z;
        }
    } else if ((abs(ekf.x(2,1)) > abs(ekf.x(3,1))) && (abs(ekf.x(2,1)) > abs(ekf.x(4,1)))) {
        if (((ekf.x(2,1) > 0) && (z(2,1) < 0)) || ((ekf.x(2,1) < 0) && (z(2,1) > 0))) {
            z = -z;
        }
    } else if ((abs(ekf.x(3,1)) > abs(ekf.x(4,1)))) {
        if (((ekf.x(3,1) > 0) && (z(3,1) < 0)) || ((ekf.x(3,1) < 0) && (z(3,1) > 0))) {
            z = -z;
        }
    } else {
        if (((ekf.x(4,1) > 0) && (z(4,1) < 0)) || ((ekf.x(4,1) < 0) && (z(4,1) > 0))) {
            z = -z;
        }
    }
    // Kalman filter correction step
    ekf.correct(z);
    //
    q(1,1) = ekf.x(1,1);
    q(2,1) = ekf.x(2,1);
    q(3,1) = ekf.x(3,1);
    q(4,1) = ekf.x(4,1);
    omega(1,1) = g(1,1)-ekf.x(5,1);
    omega(2,1) = g(2,1)-ekf.x(6,1);
    omega(3,1) = g(3,1)-ekf.x(7,1);
    q = triad.q;
}
            