#pragma once

#include "Matrix.hpp"

template <int Dim> class Vector : public Matrix<Dim, 1> {
public:
    Vector();
    Vector(const float *data);
    Vector(const Vector<Dim> &other);
    ~Vector();

    float &operator[](int index);
    const float &operator()(int index) const;

    float dot(const Vector<Dim> &other) const;
    Vector<Dim> cross(const Vector<Dim> &other) const;
    float magnitude() const;
    Vector<Dim> normalize() const;
    float angle(const Vector<Dim> &other) const;

    static Vector<Dim> Zero();

    Vector<Dim> inverse() const = delete;
    static Vector<Dim> identity() = delete;

    Vector<Dim> &operator=(const Vector<Dim> &other);
    Vector<Dim> &operator=(const float *data);

    Vector<Dim> operator+(const Vector<Dim> &other) const;
    Vector<Dim> operator-(const Vector<Dim> &other) const;
    Vector<Dim> operator*(const Vector<Dim> &other) const;
    friend Vector<Dim> operator*(const float &scalar, const Vector<Dim> &vector)
    {
        return vector * scalar;
    }
    Vector<Dim> operator*(const float &scalar) const;
    Vector<Dim> operator/(const float &scalar) const;
    Vector<Dim> operator-() const;
    Vector<Dim> operator+() const;
    Vector<Dim> &operator~() const;
    Vector<Dim> &operator+=(const Vector<Dim> &other);
    Vector<Dim> &operator-=(const Vector<Dim> &other);
    Vector<Dim> &operator*=(const float &scalar);
    Vector<Dim> &operator/=(const float &scalar);

    operator float *();
};

template <int Dim>
Vector<Dim>::Vector()
    : Matrix<Dim, 1>()
{
}

template <int Dim>
Vector<Dim>::Vector(const float *data)
    : Matrix<Dim, 1>(data)
{
}

template <int Dim>
Vector<Dim>::Vector(const Vector<Dim> &other)
    : Matrix<Dim, 1>(other)
{
}

template <int Dim> Vector<Dim>::~Vector()
{
}

template <int Dim> float &Vector<Dim>::operator[](int index)
{
    return this->data[index];
}

template <int Dim> const float &Vector<Dim>::operator()(int index) const
{
    return this->data[index];
}

template <int Dim> float Vector<Dim>::dot(const Vector<Dim> &other) const
{
    float result;
    arm_dot_prod_f32(this->data, other.data, Dim, &result);
    return result;
}

template <int Dim>
Vector<Dim> Vector<Dim>::cross(const Vector<Dim> &other) const
{
    Vector<Dim> result;
    for (int i = 0; i < Dim; i++) {
        result[i] = this->data[(i + 1) % Dim] * other.data[(i + 2) % Dim] -
                    this->data[(i + 2) % Dim] * other.data[(i + 1) % Dim];
    }
    return result;
}

template <int Dim> float Vector<Dim>::magnitude() const
{
    float result;
    arm_power_f32(this->data, Dim, &result);
    arm_sqrt_f32(result, &result);
    return result;
}

template <int Dim> Vector<Dim> Vector<Dim>::normalize() const
{
    Vector<Dim> result;
    arm_scale_f32(this->data, 1.0f / this->magnitude(), result.data, Dim);
    return result;
}

template <int Dim> float Vector<Dim>::angle(const Vector<Dim> &other) const
{
    return acosf(this->dot(other) / (this->magnitude() * other.magnitude()));
}

template <int Dim> Vector<Dim> Vector<Dim>::Zero()
{
    Vector<Dim> result;
    arm_fill_f32(0.0f, result.data, Dim);
    return result;
}

template <int Dim> Vector<Dim> &Vector<Dim>::operator=(const Vector<Dim> &other)
{
    arm_copy_f32(other.data, this->data, Dim);
    return *this;
}

template <int Dim> Vector<Dim> &Vector<Dim>::operator=(const float *data)
{
    arm_copy_f32(data, this->data, Dim);
    return *this;
}

template <int Dim>
Vector<Dim> Vector<Dim>::operator+(const Vector<Dim> &other) const
{
    Vector<Dim> result;
    arm_add_f32(this->data, other.data, result.data, Dim);
    return result;
}

template <int Dim>
Vector<Dim> Vector<Dim>::operator-(const Vector<Dim> &other) const
{
    Vector<Dim> result;
    arm_sub_f32(this->data, other.data, result.data, Dim);
    return result;
}

template <int Dim>
Vector<Dim> Vector<Dim>::operator*(const Vector<Dim> &other) const
{
    return this->cross(other);
}

template <int Dim> Vector<Dim> Vector<Dim>::operator*(const float &scalar) const
{
    Vector<Dim> result;
    arm_scale_f32(this->data, scalar, result.data, Dim);
    return result;
}

template <int Dim> Vector<Dim> Vector<Dim>::operator/(const float &scalar) const
{
    Vector<Dim> result;
    arm_scale_f32(this->data, 1.0f / scalar, result.data, Dim);
    return result;
}

template <int Dim> Vector<Dim> Vector<Dim>::operator-() const
{
    Vector<Dim> result;
    arm_negate_f32(this->data, result.data, Dim);
    return result;
}

template <int Dim> Vector<Dim> Vector<Dim>::operator+() const
{
    return *this;
}

template <int Dim> Vector<Dim> &Vector<Dim>::operator~() const
{
    return this->inverse();
}

template <int Dim>
Vector<Dim> &Vector<Dim>::operator+=(const Vector<Dim> &other)
{
    arm_add_f32(this->data, other.data, this->data, Dim);
    return *this;
}

template <int Dim>
Vector<Dim> &Vector<Dim>::operator-=(const Vector<Dim> &other)
{
    arm_sub_f32(this->data, other.data, this->data, Dim);
    return *this;
}

template <int Dim> Vector<Dim> &Vector<Dim>::operator*=(const float &scalar)
{
    arm_scale_f32(this->data, scalar, this->data, Dim);
    return *this;
}

template <int Dim> Vector<Dim> &Vector<Dim>::operator/=(const float &scalar)
{
    arm_scale_f32(this->data, 1.0f / scalar, this->data, Dim);
    return *this;
}

template <int Dim> Vector<Dim>::operator float *()
{
    return this->data;
}

using Vector2f = Vector<2>;
using Vector3f = Vector<3>;
using Vector4f = Vector<4>;
using Vector5f = Vector<5>;
using Vector6f = Vector<6>;
using Vector7f = Vector<7>;
using Vector8f = Vector<8>;
using Vector9f = Vector<9>;