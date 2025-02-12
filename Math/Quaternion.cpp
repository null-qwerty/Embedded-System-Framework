#include "Math/Quaternion.hpp"
#include "Math/Matrix.hpp"

Quaternion::Quaternion()
    : Vector<4>()
{
}

Quaternion::Quaternion(const float *data)
    : Vector<4>(data)
{
}

Quaternion::Quaternion(const Quaternion &other)
    : Vector<4>(other)
{
}

Quaternion::Quaternion(const Vector4f &other)
    : Vector<4>(other)
{
}

Quaternion::Quaternion(float w, float x, float y, float z)
{
    this->w() = w;
    this->x() = x;
    this->y() = y;
    this->z() = z;
}

Quaternion::~Quaternion()
{
}

Quaternion Quaternion::identity()
{
    Quaternion result;
    result.w() = 1.0f;
    result.x() = 0.0f;
    result.y() = 0.0f;
    result.z() = 0.0f;
    return result;
}

Quaternion Quaternion::conjugate() const
{
    Quaternion result;
    result.w() = this->W();
    result.x() = -this->X();
    result.y() = -this->Y();
    result.z() = -this->Z();
    return result;
}

Quaternion Quaternion::inverse() const
{
    Quaternion result;
    float magnitude = this->magnitude();
    result.w() = this->W() / magnitude;
    result.x() = -this->X() / magnitude;
    result.y() = -this->Y() / magnitude;
    result.z() = -this->Z() / magnitude;
    return result;
}

Quaternion &Quaternion::fromRotationMatrix(const Matrix3x3f &matrix)
{
    float trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
    if (trace > 0.0f) {
        float s = 0.5f / sqrtf(trace + 1.0f);
        this->w() = 0.25f / s;
        this->x() = (matrix(2, 1) - matrix(1, 2)) * s;
        this->y() = (matrix(0, 2) - matrix(2, 0)) * s;
        this->z() = (matrix(1, 0) - matrix(0, 1)) * s;
    } else {
        if (matrix(0, 0) > matrix(1, 1) && matrix(0, 0) > matrix(2, 2)) {
            float s =
                2.0f * sqrtf(1.0f + matrix(0, 0) - matrix(1, 1) - matrix(2, 2));
            this->w() = (matrix(2, 1) - matrix(1, 2)) / s;
            this->x() = 0.25f * s;
            this->y() = (matrix(0, 1) + matrix(1, 0)) / s;
            this->z() = (matrix(0, 2) + matrix(2, 0)) / s;
        } else if (matrix(1, 1) > matrix(2, 2)) {
            float s =
                2.0f * sqrtf(1.0f + matrix(1, 1) - matrix(0, 0) - matrix(2, 2));
            this->w() = (matrix(0, 2) - matrix(2, 0)) / s;
            this->x() = (matrix(0, 1) + matrix(1, 0)) / s;
            this->y() = 0.25f * s;
            this->z() = (matrix(1, 2) + matrix(2, 1)) / s;
        } else {
            float s =
                2.0f * sqrtf(1.0f + matrix(2, 2) - matrix(0, 0) - matrix(1, 1));
            this->w() = (matrix(1, 0) - matrix(0, 1)) / s;
            this->x() = (matrix(0, 2) + matrix(2, 0)) / s;
            this->y() = (matrix(1, 2) + matrix(2, 1)) / s;
            this->z() = 0.25f * s;
        }
    }
    return *this;
}

Quaternion &Quaternion::fromEulerAngles(float yaw, float pitch, float roll)
{
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    this->w() = cr * cp * cy + sr * sp * sy;
    this->x() = sr * cp * cy - cr * sp * sy;
    this->y() = cr * sp * cy + sr * cp * sy;
    this->z() = cr * cp * sy - sr * sp * cy;

    return *this;
}

Matrix3x3f Quaternion::toRotationMatrix() const
{
    Matrix3x3f result;

    float xx = this->X() * this->X();
    float xy = this->X() * this->Y();
    float xz = this->X() * this->Z();
    float xw = this->X() * this->W();
    float yy = this->Y() * this->Y();
    float yz = this->Y() * this->Z();
    float yw = this->Y() * this->W();
    float zz = this->Z() * this->Z();
    float zw = this->Z() * this->W();

    result(0, 0) = 1.0f - 2.0f * (yy + zz);
    result(0, 1) = 2.0f * (xy - zw);
    result(0, 2) = 2.0f * (xz + yw);
    result(1, 0) = 2.0f * (xy + zw);
    result(1, 1) = 1.0f - 2.0f * (xx + zz);
    result(1, 2) = 2.0f * (yz - xw);
    result(2, 0) = 2.0f * (xz - yw);
    result(2, 1) = 2.0f * (yz + xw);
    result(2, 2) = 1.0f - 2.0f * (xx + yy);

    return result;
}

Vector3f Quaternion::toEulerAngles() const
{
    Vector3f result;

    float roll =
        atan2f(2 * (this->W() * this->X() + this->Y() * this->Z()),
               1 - 2 * (this->X() * this->X() + this->Y() * this->Y()));
    float pitch = asinf(2 * (this->W() * this->Y() - this->Z() * this->X()));
    float yaw = atan2f(2 * (this->W() * this->Z() + this->X() * this->Y()),
                       1 - 2 * (this->Y() * this->Y() + this->Z() * this->Z()));

    result << roll, pitch, yaw;

    return result;
}

Quaternion Quaternion::operator*(const Quaternion &other)
{
    Quaternion res;

    res.w() = this->W() * other.W() - this->X() * other.X() -
              this->Y() * other.Y() - this->Z() * other.Z();
    res.x() = this->W() * other.X() + this->X() * other.W() +
              this->Y() * other.Z() - this->Z() * other.Y();
    res.y() = this->W() * other.Y() - this->X() * other.Z() +
              this->Y() * other.W() + this->Z() * other.X();
    res.z() = this->W() * other.Z() + this->X() * other.Y() -
              this->Y() * other.X() + this->Z() * other.W();

    return res;
}

Quaternion Quaternion::operator*(const float &scalar)
{
    Quaternion res;

    res.w() = this->W() * scalar;
    res.x() = this->X() * scalar;
    res.y() = this->Y() * scalar;
    res.z() = this->Z() * scalar;

    return *this;
}

float &Quaternion::w()
{
    return this->data[0];
}

float &Quaternion::x()
{
    return this->data[1];
}

float &Quaternion::y()
{
    return this->data[2];
}

float &Quaternion::z()
{
    return this->data[3];
}

float Quaternion::W() const
{
    return this->data[0];
}

float Quaternion::X() const
{
    return this->data[1];
}

float Quaternion::Y() const
{
    return this->data[2];
}

float Quaternion::Z() const
{
    return this->data[3];
}

Vector3f Quaternion::XYZ() const
{
    Vector3f result;
    result[0] = this->X();
    result[1] = this->Y();
    result[2] = this->Z();
    return result;
}

Quaternion::operator float *()
{
    return this->data;
}