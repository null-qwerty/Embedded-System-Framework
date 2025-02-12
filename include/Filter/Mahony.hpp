#pragma once

#include "Math/Math.hpp"
#include "Math/Vector.hpp"

class Mahony9Axis {
public:
    Mahony9Axis(float sampleFreq, float kp, float ki);
    ~Mahony9Axis() = default;

    Mahony9Axis &init(float tau);
    Quaternion update(Vector3f &gyro, Vector3f &accel, Vector3f &magn);

private:
    float sampleFreq;
    float kp;
    float ki;

    Quaternion q = Quaternion::identity();
    Vector3f errorInt = Vector3f::Zero();
};

class Mahony6Axis {
public:
    Mahony6Axis(float sampleFreq, float kp = 10.f, float ki = 0.01f);
    Mahony6Axis &init(float tau);
    ~Mahony6Axis() = default;

    Quaternion update(Vector3f gyro, Vector3f accel);

private:
    float sampleFreq;
    float kp;
    float ki;

    Quaternion q = Quaternion::identity();
    Vector3f errorInt = Vector3f::Zero();
};