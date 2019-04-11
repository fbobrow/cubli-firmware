#ifndef triad_h
#define triad_h

#include "mbed.h"
#include "utils/matrix.h"

// Triad class
class Triad
{
  public:
    // Class constructor
    Triad();
    // Update step
    void update(const Matrix& u_B, const Matrix& v_B);
    // Orientation quaternion
    Matrix q;
  private:
    // Triad method
    Matrix triad(const Matrix& u, const Matrix &v);
    // Convert direct cossine matrix to quaternions
    Matrix dcm2quat(const Matrix& R);
    // Direct cossine matrix from inertial reference frame to triad reference frame
    Matrix R_TI;
};

#endif