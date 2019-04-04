#ifndef matrix_h
#define matrix_h

#include "stdlib.h"
#include "math.h"

// Matrix class
class Matrix
{
    public:
        // Constructor
        Matrix(int r = 1, int c = 1);
        // Destructor 
        ~Matrix();
        // Assignment 
        Matrix& operator=(const Matrix& A);
        // Cell data 
        float& operator()(int r, int c);
        // Parameters 
        int rows, cols;
        float **data;
    private:
        // Memmory managment
        void allocate_memmory();
        void deallocate_memmory();
};

// Math operators 
Matrix operator+(const Matrix& A, const Matrix& B);
Matrix operator-(const Matrix& A, const Matrix& B);
Matrix operator-(const Matrix& A);
Matrix operator*(const Matrix& A, const Matrix& B);
Matrix operator*(float k, const Matrix& B);
Matrix operator*(const Matrix& A, float k);
Matrix operator/(const Matrix& A, float k);

// Matriz algebra 
Matrix eye(int r, int c = 0);
Matrix zeros(int r, int c = 0);
Matrix transpose(const Matrix& A);
Matrix inverse(const Matrix& A);
float trace(const Matrix& A);

// Vector algebra
Matrix cross(const Matrix& u, const Matrix& v);
float norm(const Matrix& u);

// Orientation algebra
Matrix dcm2quat(const Matrix& R);
Matrix triad(const Matrix& u, const Matrix& v);

#endif

