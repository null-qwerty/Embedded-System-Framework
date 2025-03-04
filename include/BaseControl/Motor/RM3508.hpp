#pragma once

#include "Motor.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"

class RM3508 : public Motor {
public:
#ifdef __CAN_H_
    RM3508(CAN &can, uint16_t send_id, uint16_t receive_id, int8_t cw);
#endif
#ifdef __FDCAN_H__
    RM3508(FDCAN &fdcan, uint16_t sendid, uint16_t receive_id, int8_t cw);
#endif
    ~RM3508();

    RM3508 &init() final;

    RM3508 &deInit() final;
    /**
     * @brief 设置电机目标状态，位置、速度、力矩，温度不做设置
     *
     * @param target_state 电机目标状态
     * @return MotorState 电机状态
     *
     * @note 会将状态值写入发送缓冲区，但不会发送
     */
    RM3508 &encodeControlMessage() final;
    /**
     * @brief 解码反馈信息
     *
     * @param data CAN 接收数据
     * @return Motor 电机
     *
     * @note 会将状态值写入电机状态
     */
    RM3508 &decodeFeedbackMessage() final;

private:
    const float MAX_CURRENT = 20.0f;
    const uint16_t MAX_POISION_DATA = 8191;
    const uint16_t MAX_CURRENT_DATA = 16384;

    float calculateControlData() final;

    RM3508 &encodeCAN();
    RM3508 &decodeCAN();
    RM3508 &encodeFDCAN();
    RM3508 &decodeFDCAN();
};
