#pragma once

/**
 * @brief class pidController
 * pid 控制器，使用 TI 开源 pid算法。
 * 若使用无参数构造函数需要手动对参数进行初始化。
 *
 * 接口定义：
 * pidController::pidController() 无参数构造函数
 * pidController::pidController(float,float,float,float,float,float,float,float)
 * 有参数构造函数，设置 Kp, Ki, Kd, Umax, Umin, Kr, Km, Kiae
 * pidController::init() 初始化，将中间变量赋 0
 *
 * pidController::setFilter(float,float) 设置微分低通滤波器
 * pidController::setPidParam(float,float,float,float,float) 设置 pid 参数
 * pidController::setOutLimit(float,float) 设置输出限幅
 * pidController::setIaeParam(float) 设置 Iae 增益
 *
 * pidController::getOut() 获取输出
 * pidController::getIae() 获取积分绝对误差 (IAE)
 *
 * pidController::Calculate(float,float) 计算控制量
 */
class pidController {
public:
    /**
     * @brief 默认构造函数
     *
     */
    pidController();
    /**
     * @brief PID 控制器构造函数
     *
     * @param Kp 比例增益
     * @param Ki 积分增益
     * @param Kd 微分增益
     * @param Umax 输出上限
     * @param Umin 输出下限
     * @param Kiae IAE 系数
     * @param Kr 参考值权重
     * @param Km 微分权重
     *
     * * @note 设置滤波器系数需要调用 setFilter 函数，默认 c1 = 1.0, c2 = 0.0
     */
    pidController(float Kp, float Ki, float Kd, float Umax, float Umin,
                  float Kr = 1.0, float Km = 1.0, float Kiae = 0.0);
    /**
     * @brief PID 控制器析构函数
     */
    ~pidController() = default;

    pidController &init();
    /**
     * @brief PID 控制器更新
     *
     * @param Ref 目标值
     * @param Fbk 当前值
     * @return float 输出值
     */
    float Calculate(float Ref, float Fbk);
    /**
     * @brief 设置滤波器系数
     *
     * @param a 截止频率 (Hz)
     * @param T 采样周期 (s)
     * @return pidController& 自身引用
     */
    pidController &setFilter(float a, float T);

    /**
     * @brief 设置 pid 参数
     *
     * @param Kp 比例系数
     * @param Ki 积分系数
     * @param Kd 微分系数
     * @param Kr 参考值权重，default = 1.0
     * @param Km 微分权重，default = 1.0
     * @return pidController& 自身引用
     */
    pidController &setPidParam(float Kp, float Ki, float Kd, float Kr = 1.0,
                               float Km = 1.0);

    /**
     * @brief 设置输出限幅
     *
     * @param Umax 上限
     * @param Umin 下限
     * @return pidController& 自身引用
     */
    pidController &setOutLimit(float Umax, float Umin);

    /**
     * @brief 设置 iae 系数
     *
     * @param Kiae iae 系数
     * @return pidController& 自身引用
     */
    pidController &setIaeParam(float Kiae);

    float getOut()
    {
        return Out;
    }

    float getIae()
    {
        return Iae;
    }

private:
    // PID param
    float Kr = 1.; // 参考值权重
    float Kp = 0.; // 比例增益
    float Ki = 0.; // 积分增益
    float Kd = 0.; // 微分增益
    float Km = 1.; // 微分权重
    float Umax = 0.; // 输出上限
    float Umin = 0.; // 输出下限
    float Kiae = 0.; // IAE 系数
    // PID terminals
    float c1 = 1.;
    float c2 = 0.; // 微分滤波器系数
    // PID data
    float up = 0; // 比例输出
    float ui = 0; // 积分输出
    float ud = 0; // 微分输出
    float v1 = 0; // 限幅前输出
    float i1 = 0; // ui(k-1)
    float d0 = 0; // ud(k)
    float d01 = 0; // d0(k-1)
    float d1 = 0; // ud(k-1)
    float d2 = 0; // d2(k-1)
    int w1 = 0; // 限幅标志 [u(k-1) - v(k-1)]

    float Out = 0; // 输出
    float Iae = 0; // IAE, 积分绝对误差
};
