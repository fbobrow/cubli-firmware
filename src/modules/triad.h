#ifndef triad_h
#define triad_h

#include "mbed.h"
#include "utils/matrix.h"

// Attitude estimator class
class Triad
{
  public:
    // 
    Triad();
    // 
    void update(const Matrix& u, const Matrix& v);
    //
    Matrix q;
  private:
    //
    Matrix triad(const Matrix& u, const Matrix &v);
    //
    Matrix dcm2quat(const Matrix& R);
    //
    Matrix NT, NT_T;
};

#endif