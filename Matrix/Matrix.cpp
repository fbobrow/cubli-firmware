#include "Matrix.h"

Matrix::Matrix()
{
    // Set matrix size
    rows = 1;
    cols = 1;
    // Allocate memory
    allocate_memmory();
    // Initialize data
    data[0][0] = 0.0f;
}

Matrix::Matrix(int r, int c)
{
    // Set matrix size
    rows = r;
    cols = c;
    // Allocate memory
    allocate_memmory();
    // Initialize data
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            data[i][j] = 0.0f;
        }
    }
}

Matrix::~Matrix()
{
    // Free memory
    deallocate_memmory();
}

Matrix& Matrix::operator=(const Matrix& m)
{
    // Re-allocate memmory (in case size is different)
    if (rows != m.rows || cols != m.cols) {
        deallocate_memmory();
        rows = m.rows;
        cols = m.cols;
        allocate_memmory();
    }
    // Copy data
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            data[i][j] = m.data[i][j];
        }
    }
    return *this;
}

float& Matrix::operator()(int r, int c)
{
    // Return cell data
    return data[r-1][c-1];
}

void Matrix::allocate_memmory()
{
    data = (float **)malloc(rows * sizeof(float *));
    for (int i = 0; i < rows; i++) {
        data[i] = (float *)malloc(cols * sizeof(float));
    }
}

void Matrix::deallocate_memmory()
{
    for (int i = 0; i < rows; i++) {
        free(data[i]);
    }
    free(data);
}

Matrix operator+(const Matrix& A, const Matrix& B)
{
    // Auxiliary matrix where C = A+B
    Matrix C(A.rows,A.cols);
    for (int i = 0; i < C.rows; i++) {
        for (int j = 0; j < C.cols; j++) {
            C.data[i][j] = A.data[i][j]+B.data[i][j];
        }
    }
    return C;
}

Matrix operator-(const Matrix& A, const Matrix& B)
{
    // Auxiliary matrix where C = A-B
    Matrix C(A.rows,A.cols);
    for (int i = 0; i < C.rows; i++) {
        for (int j = 0; j < C.cols; j++) {
            C.data[i][j] = A.data[i][j]-B.data[i][j];
        }
    }
    return C;
}

Matrix operator-(const Matrix& A)
{
    // Auxiliary matrix where C = -A
    Matrix C(A.rows,A.cols);
    for (int i = 0; i < C.rows; i++) {
        for (int j = 0; j < C.cols; j++) {
            C.data[i][j] = -A.data[i][j];
        }
    }
    return C;
}

Matrix operator*(const Matrix& A, const Matrix& B)
{
    // Auxiliary matrix where C = A*B
    Matrix C(A.rows,B.cols);
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < B.cols; j++) {
            for (int k = 0; k < A.cols; k++) {
                if((A.data[i][k] != 0.0f) && (B.data[k][j] != 0.0f)) {
                    if(A.data[i][k] == 1.0f) {
                        C.data[i][j] += B.data[k][j];
                    } else if (B.data[k][j] == 0.0f) {
                        C.data[i][j] += A.data[i][k];
                    } else {
                        C.data[i][j] += A.data[i][k]*B.data[k][j];
                    }
                }
            }
        }
    }
    return C;
}

Matrix operator*(float k, const Matrix& A)
{
    // Auxiliary matrix where B = k*A
    Matrix B(A.rows,A.cols);
    for (int i = 0; i < B.rows; i++) {
        for (int j = 0; j < B.cols; j++) {
            B.data[i][j] = k*A.data[i][j];
        }
    }
    return B;
}

Matrix operator*(const Matrix& A, float k)
{
    // Auxiliary matrix where B = A*k
    Matrix B(A.rows,A.cols);
    for (int i = 0; i < B.rows; i++) {
        for (int j = 0; j < B.cols; j++) {
            B.data[i][j] = A.data[i][j]*k;
        }
    }
    return B;
}

Matrix operator/(const Matrix& A, float k)
{
    // Auxiliary matrix where B = A/k
    Matrix B(A.rows,A.cols);
    for (int i = 0; i < B.rows; i++) {
        for (int j = 0; j < B.cols; j++) {
            B.data[i][j] = A.data[i][j]/k;
        }
    }
    return B;
}

Matrix eye(int r, int c)
{
    if (c == 0) {
        c = r;
    }
    Matrix m(r, c);
    for (int i = 0; i < m.rows; i++) {
        for (int j = 0; j < m.cols; j++) {
            if (i == j) {
                m.data[i][j] = 1.0f;
            }
        }
    }
    return m;
}

Matrix zeros(int r, int c)
{
    if (c == 0) {
        c = r;
    }
    Matrix m(r, c);
    return m;
}

Matrix transpose(const Matrix& A)
{
    // Auxiliary matrix where B = A'
    Matrix B(A.cols, A.rows);
    for (int i = 0; i < B.rows; i++) {
        for (int j = 0; j < B.cols; j++) {
            B.data[i][j] = A.data[j][i];
        }
    }
    return B;
}

Matrix inverse(const Matrix& A)
{
    // Apply A = LDL' factorization where L is a lower triangular matrix and D
    // is a block diagonal matrix
    Matrix L(A.rows, A.cols);
    Matrix D(A.rows, A.cols);
    float L_sum;
    float D_sum;
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < A.rows; j++) {
            if (i >= j) {
                if (i == j) {
                    L.data[i][j] = 1.0f;
                    D_sum = 0.0f;
                    for (int k = 0; k <= (i-1); k++) {
                        D_sum += L.data[i][k]*L.data[i][k]*D.data[k][k];
                    }
                    D.data[i][j] = A.data[i][j] - D_sum;
                } else {
                    L_sum = 0.0f;
                    for (int k = 0; k <= (j-1); k++) {
                        L_sum += L.data[i][k]*L.data[j][k]*D.data[k][k];
                    }
                    L.data[i][j] = (1.0f/D.data[j][j])*(A.data[i][j]-L_sum);
                }
            }
        }
    }
    // Compute the inverse of L and D matrices
    Matrix L_inv(A.rows, A.cols);
    Matrix D_inv(A.rows, A.cols);
    float L_inv_sum;
    for (int i = 0; i < A.rows; i++) {
        for (int j = 0; j < A.rows; j++) {
            if (i >= j) {
                if (i == j) {
                    L_inv.data[i][j] = 1.0f/L.data[i][j];
                    D_inv.data[i][j] = 1.0f/D.data[i][j];
                } else {
                    L_inv_sum = 0.0f;
                    for (int k = j; k <= (i-1); k++) {
                        L_inv_sum += L.data[i][k]*L_inv.data[k][j];
                    }
                    L_inv.data[i][j] = -L_inv_sum;
                }
            }
        }
    }
    // Compute the inverse of A matrix in terms of the inverse of L and D
    // matrices
    return transpose(L_inv)*D_inv*L_inv;
}

float trace(const Matrix& A)
{
    float t = 0.0f;
    for (int i = 0; i < A.rows; i++) {
        t += A.data[i][i];
    }
    return t;
}

Matrix cross(const Matrix& u, const Matrix& v)
{
    // Auxiliary matrix where w = uXv
    Matrix w(u.rows,u.cols);
    w.data[0][0] = u.data[1][0]*v.data[2][0]-u.data[2][0]*v.data[1][0];
    w.data[1][0] = u.data[2][0]*v.data[0][0]-u.data[0][0]*v.data[2][0];
    w.data[2][0] = u.data[0][0]*v.data[1][0]-u.data[1][0]*v.data[0][0];
    return w;
}

float norm(const Matrix& A)
{
    float n = 0.0f;
    for (int i = 0; i < A.rows; i++) {
        n += A.data[i][0]*A.data[i][0];
    }
    return sqrt(n);
}

Matrix dcm2quat(const Matrix& R)
{
    Matrix q(4, 1);
    float tr = trace(R);
    if (tr > 0.0f) {
        float sqtrp1 = sqrt( tr + 1.0f);
        q.data[0][0] = 0.5f*sqtrp1;
        q.data[1][0] = (R.data[1][2] - R.data[2][1])/(2.0f*sqtrp1);
        q.data[2][0] = (R.data[2][0] - R.data[0][2])/(2.0f*sqtrp1);
        q.data[3][0] = (R.data[0][1] - R.data[1][0])/(2.0f*sqtrp1);
    } else {
        if ((R.data[1][1] > R.data[0][0]) && (R.data[1][1] > R.data[2][2])) {
            float sqdip1 = sqrt(R.data[1][1] - R.data[0][0] - R.data[2][2] + 1.0f );
            q.data[2][0] = 0.5f*sqdip1;
            if ( sqdip1 != 0.0 ) {
                sqdip1 = 0.5f/sqdip1;
            }
            q.data[0][0] = (R.data[2][0] - R.data[0][2])*sqdip1;
            q.data[1][0] = (R.data[0][1] + R.data[1][0])*sqdip1;
            q.data[3][0] = (R.data[1][2] + R.data[2][1])*sqdip1;
        } else if (R.data[2][2] > R.data[0][0]) {
            float sqdip1 = sqrt(R.data[2][2] - R.data[0][0] - R.data[1][1] + 1.0f );
            q.data[3][0] = 0.5f*sqdip1;
            if ( sqdip1 != 0.0 ) {
                sqdip1 = 0.5f/sqdip1;
            }
            q.data[0][0] = (R.data[0][1] - R.data[1][0])*sqdip1;
            q.data[1][0] = (R.data[2][0] + R.data[0][2])*sqdip1;
            q.data[2][0] = (R.data[1][2] + R.data[2][1])*sqdip1;
        } else {
            float sqdip1 = sqrt(R.data[0][0] - R.data[1][1] - R.data[2][2] + 1.0f );
            q.data[1][0] = 0.5f*sqdip1;
            if ( sqdip1 != 0.0 ) {
                sqdip1 = 0.5f/sqdip1;
            }
            q.data[0][0] = (R.data[1][2] - R.data[2][1])*sqdip1;
            q.data[2][0] = (R.data[0][1] + R.data[1][0])*sqdip1;
            q.data[3][0] = (R.data[2][0] + R.data[0][2])*sqdip1;
        }
    }
    return q;
}