#pragma once

#include "Motor.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"

class RM3508 : public Motor {
public:
    RM3508(Connectivity &connectivity, uint16_t sendid, uint16_t receive_id,
           int8_t cw, float ratio = 3591.0f / 187.0f);
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
    const float ratio_0 = 3591.0f / 187.0f; ///< 3508 默认减速比
    float last_position = 9999.0f;
    int8_t count = 0;

    float calculateControlData() final;

    RM3508 &encodeCanData(uint16_t data);
    uint8_t *getCanData();
    RM3508 &encodeFdcanData(uint16_t data);
    uint8_t *getFdcanData();
};
