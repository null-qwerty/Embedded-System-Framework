#pragma once

#include "Math/Matrix.hpp"
#include "Math/Vector.hpp"

template <int StateDim, int InputDim> class lqrController {
public:
    lqrController() = delete;
    /**
     * @brief LQR 控制器构造函数
     *
     * @param A_d 状态矩阵，离散系统
     * @param B_d 输入矩阵，离散系统
     * @param Q 状态权重矩阵
     * @param R 输入权重矩阵
     */
    lqrController(const Matrix<StateDim, StateDim> &A_d,
                  const Matrix<StateDim, InputDim> &B_d,
                  const Matrix<StateDim, StateDim> &Q,
                  const Matrix<StateDim, StateDim> &R);
    /**
     * @brief LQR 控制器构造函数，由连续系统转换而来
     *
     * @param A 状态矩阵，连续系统
     * @param B 输入矩阵，连续系统
     * @param Q 状态权重矩阵
     * @param R 输入权重矩阵
     * @param T 采样周期，默认 0.001s
     */
    lqrController(const Matrix<StateDim, StateDim> &A,
                  const Matrix<StateDim, InputDim> &B,
                  const Matrix<StateDim, StateDim> &Q,
                  const Matrix<StateDim, StateDim> &R, float T = 0.001f);
    /**
     * @brief LQR 控制器析构函数
     */
    ~lqrController() = default;

    Matrix<InputDim, StateDim> &getK();
    Vector<InputDim> getU(const Vector<StateDim> &x);

private:
    // LQR param
    Matrix<StateDim, StateDim> A;
    Matrix<StateDim, InputDim> B;
    Matrix<StateDim, StateDim> Q;
    Matrix<StateDim, StateDim> R;

    Matrix<InputDim, StateDim> K;
    Matrix<StateDim, StateDim> P;

    lqrController<StateDim, InputDim> &calculateK();
};

template <int StateDim, int InputDim>
lqrController<StateDim, InputDim>::lqrController(
    const Matrix<StateDim, StateDim> &A_d,
    const Matrix<StateDim, InputDim> &B_d, const Matrix<StateDim, StateDim> &Q,
    const Matrix<StateDim, StateDim> &R)
    : A(A_d)
    , B(B_d)
    , Q(Q)
    , R(R)
{
    calculateK();
}

template <int StateDim, int InputDim>
lqrController<StateDim, InputDim>::lqrController(
    const Matrix<StateDim, StateDim> &A, const Matrix<StateDim, InputDim> &B,
    const Matrix<StateDim, StateDim> &Q, const Matrix<StateDim, StateDim> &R,
    float T)
    : Q(Q)
    , R(R)
{
    // 连续系统转离散系统
    // A_d = e^(AT)
    // B_d = A^(-1)(A_d - I)B
    // C_d = C
    // D_d = D
    // T 为采样周期
    Matrix<StateDim, StateDim> I = Matrix<StateDim, StateDim>::Identity();
    auto e_AT = (A * T).exp();

    this->A = e_AT;
    this->B = A.inverse() * (e_AT - I) * B;

    calculateK();
}

template <int StateDim, int InputDim>
Matrix<InputDim, StateDim> &lqrController<StateDim, InputDim>::getK()
{
    return K;
}

template <int StateDim, int InputDim>
lqrController<StateDim, InputDim> &
lqrController<StateDim, InputDim>::calculateK()
{
    // 计算最优控制器
    // A^TP + PA - PBR^(-1)B'P + Q = 0
    // K = R^(-1)B'P
    // 初始化 P
    P = Q;
    // 迭代求解 Riccati 方程
    int max_iter = 200;
    Matrix<StateDim, StateDim> diff;
    Matrix<StateDim, StateDim> P_next;
    for (int i = 0; i < max_iter; i++) {
        P_next = A.transpose() * P * A -
                 A.transpose() * P * B * (R.inverse() * B.transpose() * P) + Q;
        diff = P - P_next;
        if (diff.norm() < 1e-6) {
            break;
        }
        P = P_next;
    }

    // 计算 K
    K = R.inverse() * B.transpose() * P_next;

    return *this;
}

template <int StateDim, int InputDim>
Vector<InputDim>
lqrController<StateDim, InputDim>::getU(const Vector<StateDim> &x)
{
    // 计算控制量
    return -K * x;
}