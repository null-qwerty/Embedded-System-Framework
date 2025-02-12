#pragma once

#include "Math/Matrix.hpp"
#include "Vector.hpp"

class Quaternion : public Vector<4> {
public:
    Quaternion();
    Quaternion(const float *data);
    Quaternion(const Quaternion &other);
    Quaternion(const Vector4f &other);
    Quaternion(float w, float x, float y, float z);
    ~Quaternion();

    static Quaternion identity();

    Quaternion conjugate() const;
    Quaternion inverse() const;
    Quaternion &fromRotationMatrix(const Matrix3x3f &matrix);
    Quaternion &fromEulerAngles(float yaw, float pitch, float roll);

    Matrix3x3f toRotationMatrix() const;
    Vector3f toEulerAngles() const;

    Quaternion operator*(const Quaternion &other);
    Quaternion operator*(const float &scalar);

    float &w();
    float &x();
    float &y();
    float &z();

    float W() const;
    float X() const;
    float Y() const;
    float Z() const;

    Vector3f XYZ() const;

    operator float *();
};