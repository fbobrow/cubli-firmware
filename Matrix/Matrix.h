#ifndef Matrix_h
#define Matrix_h

#include "stdlib.h"
#include "math.h"

// Matrix class 
class Matrix
{
public:
    // Constructors 
    Matrix();
    Matrix(int, int);
    // Destructor 
    ~Matrix();
    // Assignment 
    Matrix& operator=(const Matrix&);
    // Cell data 
    float& operator()(int, int);
    // Parameters 
    int rows, cols;
    float **data;
private:
    // Memmory managment
    void allocate_memmory();
    void deallocate_memmory();
};

// Math operators 
Matrix operator+(const Matrix&, const Matrix&);
Matrix operator-(const Matrix&, const Matrix&);
Matrix operator-(const Matrix&);
Matrix operator*(const Matrix&, const Matrix&);
Matrix operator*(float, const Matrix&);
Matrix operator*(const Matrix&, float);
Matrix operator/(const Matrix&, float);

// Matriz algebra 
Matrix eye(int, int = 0);
Matrix zeros(int, int = 0);
Matrix transpose(const Matrix&);
Matrix inverse(const Matrix&);
float trace(const Matrix&);

// Vector algebra
Matrix cross(const Matrix&, const Matrix&);
float norm(const Matrix&);

// Orientation algebra
Matrix dcm2quat(const Matrix&);

#endif

