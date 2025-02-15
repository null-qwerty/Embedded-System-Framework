#pragma once

#include "Math/Math.hpp"
#include "Math/Quaternion.hpp"
#include "Math/Vector.hpp"

class Mahony {
public:
    Mahony(float sampleFreq, float kp = 10.f, float ki = 0.01f);
    ~Mahony() = default;

    Mahony &init(float wn);
    Quaternion update(Vector3f gyro, Vector3f accel, Vector3f magn);
    Quaternion update(Vector3f gyro, Vector3f accel);
    Quaternion yawZeroDriftOffset(float yaw);

private:
    float sampleFreq;
    float kp;
    float ki;

    Quaternion q = Quaternion::identity();
    Vector3f errorInt = Vector3f::Zero();
};
