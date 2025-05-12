#pragma once

#include "classicController.hpp"

/**
 * @brief ADRC 控制器
 *
 * @details
 * 自适应动态线性化控制器，包含跟踪微分器、扩张状态观测器和非线性状态误差反馈。
 * 该控制器适用于非线性系统的控制，具有较好的鲁棒性和适应性。
 *
 * @note
 * 该控制器的实现基于 ADRC 控制理论，具体算法和参数设置可以参考相关文献。
 * 该控制器的参数设置需要根据具体系统进行调整，以达到最佳控制效果。
 *
 * 调参时可以参考以下几点：
 * 采样周期 h: 数量级在 1e-3;
 * 快速跟踪因子 r：数量级在 1e5;
 * 系统系数 b：取 1 或 2;
 * fal 函数的线性区间 delta：5 到 10 倍的采样周期;
 * ESO 增益 beta01、beta02、beta03 有两种整定方法：
 * - 观测器带宽：设带宽为 wo, 则 b01 = 3 * wo, b02 = 3 * wo^2, b03 = wo^3;
 * - 采样周期：b01 = 1 / h, b02 = 1 / 4h^2, b03 = 1 / 64h^3;
 * 线性区间 alpha1、alpha2：一般 0 < alpha1 < 1 < alpha2;
 * 跟踪输入信号增益 beta1、beta2：相当于 kp 和 kd。
 *
 * 整定流程：先调 TD, 再调 ESO, 最后调 NLSEF
 */
class adrcController : public classicController {
public:
    // 构造函数，初始化参数
    adrcController(float h, float r, float b, float delta, float beta01,
                   float beta02, float beta03, float alpha1 = 0.5,
                   float alpha2 = 1.5, float beta1 = 1.0, float beta2 = 1.0);

    // 更新控制器状态
    virtual float calculate(float ref, float fdb) override;

    // 获取控制输出
    float getControlOutput() const
    {
        return u;
    }

private:
    /*      TD      */
    float h; // 采样周期
    float r; // 快速跟踪因子
    float x1, x2; // 跟踪微分器状态
    /*      ESO     */
    float b; // 系统系数
    float delta; // fal 函数的线性区间
    float beta01, beta02, beta03; // ESO 增益
    float z1, z2, z3; // 扩张状态观测器状态
    float e, fe, fe1;
    /*    NLSEF    */
    float alpha1, alpha2; // 非线性叠加时 fal 函数线性区间
    float beta1, beta2; // 跟踪输入信号增益

    float u; // 输出

    int sign(float x);
    float fsg(float x, float y);
    float fhan(float x1, float x2, float r, float h);
    float fal(float x, float a, float delta);
};