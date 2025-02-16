#include "Filter/Mahony.hpp"
#include "Math/Matrix.hpp"
#include "Math/Vector.hpp"

Mahony::Mahony(float sampleFreq, float kp, float ki)
    : sampleFreq(sampleFreq)
    , kp(kp)
    , ki(ki)
{
}

Mahony &Mahony::init(float tau)
{
    this->errorInt = Vector3f::Zero();

    float beta = 2.146f / tau;
    kp = 2.0f * beta;
    ki = beta * beta;

    return *this;
}

Quaternion Mahony::update(Vector3f gyro, Vector3f accel, Vector3f magn)
{
    if (magn == Vector3f::Zero()) {
        return update(gyro, accel);
    }
    // lastMagn = magn;
    Matrix3x3f R = q.toRotationMatrix();
    // Normalise accelerometer measurement
    // 归一化加速度计测量
    auto accelNorm = accel.normalize();
    // Normalise magnetometer measurement
    // 归一化磁力计测量
    auto magnNorm = magn.normalize();

    // Reference direction of Earth's magnetic field
    // 地球磁场的参考方向
    Vector3f h;
    h = R * magnNorm;
    float bx = sqrtf(h(0) * h(0) + h(1) * h(1));
    float bz = h(2);
    Vector3f w;
    w << bx, 0.0f, bz;
    w = R.transpose() * w;
    // Estimated direction of gravity and magnetic field
    // 重力和磁场的估计方向
    Vector3f v;
    // v << R(2, 0), R(2, 1), R(2, 2);
    v << 0.0f, 0.0f, 1.0f;
    v = R.transpose() * v;
    // Error is cross product between estimated direction and measured direction
    // of gravity
    // 残差是估计方向和测量方向之间的叉积
    Vector3f e_acc, e_mag;
    e_acc = accelNorm.cross(v);
    e_mag = magnNorm.cross(w);
    Vector3f error;
    error = e_acc + e_mag;
    // Integral error scaled integral gain
    // 积分误差
    this->errorInt += error * ki * (1.0f / sampleFreq);
    if (ki <= 0.0f) {
        this->errorInt = Vector3f::Zero();
    }

    // Apply feedback terms
    // 反馈项
    gyro += kp * error + this->errorInt;

    // Integrate rate of change of quaternion
    // 积分四元数的导数
    gyro *= (0.5f / sampleFreq);
    q += q * Quaternion(0.0f, gyro(0), gyro(1), gyro(2));

    // Normalise quaternion
    // 归一化四元数
    q = q.normalize();

    return q;
}

Quaternion Mahony::update(Vector3f gyro, Vector3f accel)
{
    // Normalise accelerometer measurement
    // 归一化加速度计测量
    auto accelNorm = accel.normalize();

    // Reference direction of Earth's gravity
    // 重力加速度的参考方向
    Vector3f v_gravity;
    Matrix3x3f R = q.toRotationMatrix();
    v_gravity << R(2, 0), R(2, 1), R(2, 2);

    // Error is cross product between estimated direction and measured direction
    // of gravity
    // 残差是估计方向和测量方向之间的叉积
    Vector3f error;
    error = accelNorm.cross(v_gravity);
    // Integral error scaled integral gain
    // 积分误差
    this->errorInt += error * ki * (1.0f / sampleFreq);
    if (ki <= 0.0f) {
        this->errorInt = Vector3f::Zero();
    }

    // Apply feedback terms
    // 反馈项
    gyro += kp * error + this->errorInt;

    // Integrate rate of change of quaternion
    // 积分四元数的导数
    gyro *= (0.5f / sampleFreq);
    q += q * Quaternion(0.0f, gyro(0), gyro(1), gyro(2));

    // Normalise quaternion
    // 归一化四元数
    q = q.normalize();

    return q;
}

Quaternion Mahony::yawZeroDriftOffset(float yaw)
{
    auto euler = q.toEulerAngles();
    q.fromEulerAngles(euler[2] - yaw, euler[1], euler[0]);
    q.normalize();

    return q;
}