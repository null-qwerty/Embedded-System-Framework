#include "Filter/Mahony.hpp"
#include "Math/Matrix.hpp"
#include "Math/Vector.hpp"

Mahony9Axis::Mahony9Axis(float sampleFreq, float kp, float ki)
    : sampleFreq(sampleFreq)
    , kp(kp)
    , ki(ki)
{
}

Quaternion Mahony9Axis::update(Vector3f &gyro, Vector3f &accel, Vector3f &magn)
{
    float q0 = q.W(), q1 = q.X(), q2 = q.Y(), q3 = q.Z();
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // Normalise accelerometer measurement
    // 归一化加速度计测量
    auto accelNorm = accel.normalize();
    // Normalise magnetometer measurement
    // 归一化磁力计测量
    auto magnNorm = magn.normalize();

    // Reference direction of Earth's magnetic field
    // 地球磁场的参考方向
    float hx = 2.0f * magnNorm(0) * (0.5f - q2q2 - q3q3) +
               2.0f * magnNorm(1) * (q1q2 - q0q3) +
               2.0f * magnNorm(2) * (q1q3 + q0q2);
    float hy = 2.0f * magnNorm(0) * (q1q2 + q0q3) +
               2.0f * magnNorm(1) * (0.5f - q1q1 - q3q3) +
               2.0f * magnNorm(2) * (q2q3 - q0q1);
    float bx = sqrtf((hx * hx) + (hy * hy));
    float bz = 2.0f * magnNorm(0) * (q1q3 - q0q2) +
               2.0f * magnNorm(1) * (q2q3 + q0q1) +
               2.0f * magnNorm(2) * (0.5f - q1q1 - q2q2);

    // Estimated direction of gravity and magnetic field
    // 重力和磁场的估计方向
    float vx = 2.0f * (q1q3 - q0q2);
    float vy = 2.0f * (q0q1 + q2q3);
    float vz = q0q0 - q1q1 - q2q2 + q3q3;
    float wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    float wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    float wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

    // Error is cross product between estimated direction and measured direction
    // of gravity
    // 残差是估计方向和测量方向之间的叉积
    float ex = (accelNorm(1) * vz - accelNorm(2) * vy) +
               (magnNorm(1) * wz - magnNorm(2) * wy);
    float ey = (accelNorm(2) * vx - accelNorm(0) * vz) +
               (magnNorm(2) * wx - magnNorm(0) * wz);
    float ez = (accelNorm(0) * vy - accelNorm(1) * vx) +
               (magnNorm(0) * wy - magnNorm(1) * wx);
    Vector3f error;
    error << ex, ey, ez;
    // Integral error scaled integral gain
    // 积分误差
    this->errorInt += error * ki;
    if (ki <= 0.0f) {
        this->errorInt = Vector3f::Zero();
    }

    // Apply feedback terms
    // 反馈项
    gyro += kp * error + this->errorInt;

    // Integrate rate of change of quaternion
    // 积分四元数的导数
    q += q * Quaternion(0.0f, gyro(0), gyro(1), gyro(2)) * (0.5f / sampleFreq);

    // Normalise quaternion
    // 归一化四元数
    q = q.normalize();

    return q;
}

Mahony6Axis::Mahony6Axis(float sampleFreq, float kp, float ki)
    : sampleFreq(sampleFreq)
    , kp(kp)
    , ki(ki)
{
}

Mahony6Axis &Mahony6Axis::init(float tau)
{
    this->errorInt = Vector3f::Zero();

    float beta = 1.0f / tau;
    this->kp = 2.0f * beta;
    this->ki = beta * beta;

    return *this;
}

Quaternion Mahony6Axis::update(Vector3f gyro, Vector3f accel)
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

    // 消除 yaw 角零漂
    auto euler = q.toEulerAngles();
    q.fromEulerAngles(euler[2] - 2.6381924E-6, euler[1], euler[0]);

    // Normalise quaternion
    // 归一化四元数
    q = q.normalize();

    return q;
}
