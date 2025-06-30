#pragma once

#include "BaseControl/Connectivity/Connectivity.hpp"
#include "BaseControl/Controller/classicController.hpp"

/**
 * @brief class Motor
 * 封装了控制电机所需的内容，使用前需初始化其中的控制器，抽象类
 *
 * 接口定义：
 * Motor::Motor(Connectivity,uint16_t,uint16_t) 构造函数
 *
 * Motor::getState() 获取电机反馈值状态结构
 * Motor::getTargetState() 获取电机设定值状态结构
 * Motor::getAngleLoopController() 获取角度环控制器，用于初始化控制器
 * Motor::getSpeedLoopController() 获取速度环控制器，用于初始化控制器
 * Motor::getCurrentLoopController() 获取电流环控制器，用于初始化控制器
 * Motor::getId() 获取电机 ID
 *
 * Motor::encodeControlMessage() 从设定值构建控制消息 Data 数据段，纯虚函数
 * Motor::decodeFeedbackMessage(uint8_t*)
 * 从反馈信息 Data 数据段解析当前状态，纯虚函数
 */

class Motor {
public:
    typedef struct {
        float position; ///< 位置
        float velocity; ///< 速度
        float toreque; ///< 力矩，电流
        float temprature; ///< 温度
    } MotorState;

    /**
     * @brief Motor 构造函数
     *
     * @param connectivity 通信总线
     * @param send_id 发送 ID
     * @param receive_id 接收 ID
     */
    Motor(Connectivity &connectivity, uint16_t send_id, uint16_t receive_id);
    /**
     * @brief Motor 析构函数
     *
     */
    ~Motor() = default;

    virtual Motor &init() = 0;

    virtual Motor &deInit() = 0;

    bool ifInitialed();
    /**
     * @brief 获取电机当前状态
     *
     * @return MotorState 电机状态
     */
    MotorState &getState();

    /**
     * @brief 获取电机目标状态
     *
     * @return MotorState 电机状态
     */
    MotorState &getTargetState();

    /**
     * @brief 获取角度环控制器
     *
     * @return pidController& 角度环控制器
     */
    classicController *&getAngleLoopController();
    /**
     * @brief 获取速度环控制器
     *
     * @return pidController& 速度环控制器
     */
    classicController *&getSpeedLoopController();
    /**
     * @brief 获取电流环控制器
     *
     * @return pidController& 电流环控制器
     */
    classicController *&getCurrentLoopController();

    /**
     * @brief 获取电机发送 ID，用于发送数据
     *
     * @return uint8_t
     */
    uint16_t getSendId();

    /**
     * @brief 获取电机接收 ID，用于接收数据
     *
     * @return uint8_t
     */
    uint16_t getReceiveId();

    /**
     * @brief 目标状态值写入控制信息
     *
     * @return Motor& 自身引用
     *
     * @note 会将状态值写入发送缓冲区，但不会发送
     */
    virtual Motor &encodeControlMessage() = 0;

    /**
     * @brief 解码反馈信息
     *
     * @param data 接收数据
     *
     * @return Motor& 自身引用
     *
     * @note 会将状态值写入电机状态
     */
    virtual Motor &decodeFeedbackMessage() = 0;

protected:
    uint16_t send_id = 0, receive_id = 0; ///< 发送 ID 和接收 ID

    MotorState state = {}; ///< 电机状态
    MotorState targetState = {}; ///< 电机目标状态
    MotorState refState = {}; ///< 电机目标状态, 用于控制器计算

    classicController *angleLoop = nullptr;
    classicController *speedLoop = nullptr;
    classicController *currentLoop = nullptr;

    Connectivity &connectivity; ///< 通信接口，定义为引用，避免拷贝

    int8_t clockwise = 1; ///< 旋转方向
    float radio = 1.0f; ///< 传动比

    bool initialed = false;

    virtual float calculateControlData() = 0;
};

using Wheel = Motor;
using Joint = Motor;

constexpr int8_t LEFT_MOTOR_CLOCKWISE = -1; // 左边逆时针
constexpr int8_t RIGHT_MOTOR_CLOCKWISE = 1; // 右边顺时针
constexpr int8_t FRONT_MOTOR_CLOCKWISE = -1; // 前关节相对左平面逆时针
constexpr int8_t BACK_MOTOR_CLOCKWISE = 1; // 后关节相对左平面顺时针

#include "DM4310.hpp"
#include "RM3508.hpp"
#include "UnitreeA1.hpp"