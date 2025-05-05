#pragma once

#include "arm_math.h"
#include "matrix_functions.h"
#include "statistics_functions.h"

template <int Rows, int Cols> class Matrix {
public:
    Matrix();
    Matrix(const float *data);
    Matrix(const Matrix<Rows, Cols> &other);
    Matrix(const Matrix<Rows, Cols> &&other);
    ~Matrix();

    Matrix<Rows, Cols> &operator=(const Matrix<Rows, Cols> &other);
    Matrix<Rows, Cols> &operator=(const float *data);

    Matrix<Rows, Cols> operator+(const Matrix<Rows, Cols> &other) const;
    Matrix<Rows, Cols> operator-(const Matrix<Rows, Cols> &other) const;
    // Matrix<Rows, Cols> operator*(const Matrix<Rows, Cols> &other) const;
    template <int resCols>
    Matrix<Rows, resCols> operator*(const Matrix<Cols, resCols> &other);
    Matrix<Rows, Cols> operator*(const float &scalar) const;
    friend Matrix<Rows, Cols> operator*(const float &scalar,
                                        const Matrix<Rows, Cols> &matrix)
    {
        return matrix * scalar;
    }
    Matrix<Rows, Cols> operator/(const float &scalar) const;
    Matrix<Rows, Cols> operator-() const;
    Matrix<Rows, Cols> operator+() const;
    Matrix<Rows, Cols> &operator~() const;
    Matrix<Rows, Cols> &operator+=(const Matrix<Rows, Cols> &other);
    Matrix<Rows, Cols> &operator-=(const Matrix<Rows, Cols> &other);
    Matrix<Rows, Cols> &operator*=(const float &scalar);
    Matrix<Rows, Cols> &operator/=(const float &scalar);

    bool operator==(const Matrix<Rows, Cols> &other) const;

    Matrix<Rows, Cols> &operator<<(float data);
    Matrix<Rows, Cols> &operator,(float data);

    float &operator()(int row, int col);
    const float &operator()(int row, int col) const;

    Matrix<Rows, Cols> transpose() const;
    Matrix<Rows, Cols> inverse() const;
    Matrix<Rows, Cols> exp() const;

    static Matrix<Rows, Cols> identity();
    static Matrix<Rows, Cols> zero();

    static Matrix<Rows, Cols> fromArray(const float *data);
    void toArray(float *data) const;

    operator float *();

    template <int otherRows, int otherCols> friend class Matrix;

protected:
    arm_matrix_instance_f32 matrix;
    float data[Rows * Cols];
    int inputIndex = 0;
};

template <int Rows, int Cols> Matrix<Rows, Cols>::Matrix()
{
    arm_mat_init_f32(&matrix, Rows, Cols, data);
}

template <int Rows, int Cols> Matrix<Rows, Cols>::Matrix(const float *data)
{
    arm_mat_init_f32(&matrix, Rows, Cols, this->data);
    arm_copy_f32(data, this->data, Rows * Cols);
}

template <int Rows, int Cols>
Matrix<Rows, Cols>::Matrix(const Matrix<Rows, Cols> &other)
{
    arm_mat_init_f32(&matrix, Rows, Cols, data);
    arm_copy_f32(other.data, data, Rows * Cols);
}

template <int Rows, int Cols> Matrix<Rows, Cols>::~Matrix()
{
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &
Matrix<Rows, Cols>::operator=(const Matrix<Rows, Cols> &other)
{
    arm_copy_f32(other.data, data, Rows * Cols);
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &Matrix<Rows, Cols>::operator=(const float *data)
{
    arm_copy_f32(data, this->data, Rows * Cols);
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols>
Matrix<Rows, Cols>::operator+(const Matrix<Rows, Cols> &other) const
{
    Matrix<Rows, Cols> result;
    arm_mat_add_f32(&matrix, &other.matrix, &result.matrix);
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols>
Matrix<Rows, Cols>::operator-(const Matrix<Rows, Cols> &other) const
{
    Matrix<Rows, Cols> result;
    arm_mat_sub_f32(&matrix, &other.matrix, &result.matrix);
    return result;
}

template <int Rows, int Cols>
template <int resCols>
Matrix<Rows, resCols>
Matrix<Rows, Cols>::operator*(const Matrix<Cols, resCols> &other)
{
    Matrix<Rows, resCols> result;
    arm_mat_mult_f32(&matrix, &other.matrix, &result.matrix);
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::operator*(const float &scalar) const
{
    Matrix<Rows, Cols> result;
    arm_mat_scale_f32(&matrix, scalar, &result.matrix);
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::operator/(const float &scalar) const
{
    Matrix<Rows, Cols> result;
    arm_mat_scale_f32(&matrix, 1.0f / scalar, &result.matrix);
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::operator-() const
{
    Matrix<Rows, Cols> result;
    arm_negate_f32(data, &result.data);
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::operator+() const
{
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &Matrix<Rows, Cols>::operator~() const
{
    return this->inverse();
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &
Matrix<Rows, Cols>::operator+=(const Matrix<Rows, Cols> &other)
{
    arm_mat_add_f32(&matrix, &other.matrix, &matrix);
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &
Matrix<Rows, Cols>::operator-=(const Matrix<Rows, Cols> &other)
{
    arm_mat_sub_f32(&matrix, &other.matrix, &matrix);
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &Matrix<Rows, Cols>::operator*=(const float &scalar)
{
    arm_mat_scale_f32(&matrix, scalar, &matrix);
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &Matrix<Rows, Cols>::operator/=(const float &scalar)
{
    arm_mat_scale_f32(&matrix, 1.0f / scalar, &matrix);
    return *this;
}

template <int Rows, int Cols>
bool Matrix<Rows, Cols>::operator==(const Matrix<Rows, Cols> &other) const
{
    for (int i = 0; i < Rows * Cols; i++) {
        if (this->data[i] != other.data[i])
            return false;
    }
    return true;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &Matrix<Rows, Cols>::operator<<(float data)
{
    if (this->inputIndex < Rows * Cols) {
        this->data[this->inputIndex++] = data;
    }
    return *this;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> &Matrix<Rows, Cols>::operator,(float data)
{
    *this << data;
    return *this;
}

template <int Rows, int Cols>
float &Matrix<Rows, Cols>::operator()(int row, int col)
{
    return data[row * Cols + col];
}

template <int Rows, int Cols>
const float &Matrix<Rows, Cols>::operator()(int row, int col) const
{
    return data[row * Cols + col];
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::transpose() const
{
    Matrix<Rows, Cols> result;
    arm_mat_trans_f32(&matrix, &result.matrix);
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::inverse() const
{
    Matrix<Rows, Cols> result;
    arm_mat_inverse_f32(&matrix, &result.matrix);
    return result;
}

template <int Rows, int Cols> Matrix<Rows, Cols> Matrix<Rows, Cols>::exp() const
{
    Matrix<Rows, Cols> result;
    for (int i = 0; i < Rows; i++) {
        for (int j = 0; j < Cols; j++) {
            result(i, j) = pow(2.718281828459045f, this->data[i * Cols + j]);
        }
    }
    return result;
}

template <int Rows, int Cols> Matrix<Rows, Cols> Matrix<Rows, Cols>::identity()
{
    Matrix<Rows, Cols> result;
    for (int i = 0; i < Rows; i++) {
        for (int j = 0; j < Cols; j++) {
            if (i == j)
                result(i, j) = 1.0f;
            else
                result(i, j) = 0.0f;
        }
    }
    return result;
}

template <int Rows, int Cols> Matrix<Rows, Cols> Matrix<Rows, Cols>::zero()
{
    Matrix<Rows, Cols> result;
    memset(result.data, 0, Rows * Cols * sizeof(float));
    return result;
}

template <int Rows, int Cols>
Matrix<Rows, Cols> Matrix<Rows, Cols>::fromArray(const float *data)
{
    Matrix<Rows, Cols> result;
    arm_copy_f32(data, result.data, Rows * Cols);
    return result;
}

template <int Rows, int Cols>
void Matrix<Rows, Cols>::toArray(float *data) const
{
    arm_copy_f32(this->data, data, Rows * Cols);
}

template <int Rows, int Cols> Matrix<Rows, Cols>::operator float *()
{
    return data;
}

using Matrix2x2f = Matrix<2, 2>;
using Matrix3x3f = Matrix<3, 3>;
using Matrix4x4f = Matrix<4, 4>;
using Matrix5x5f = Matrix<5, 5>;
using Matrix6x6f = Matrix<6, 6>;
using Matrix7x7f = Matrix<7, 7>;
using Matrix8x8f = Matrix<8, 8>;
using Matrix9x9f = Matrix<9, 9>;