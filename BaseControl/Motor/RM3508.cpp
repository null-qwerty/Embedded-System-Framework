#include "BaseControl/Motor/RM3508.hpp"
#include "BaseControl/Motor/Motor.hpp"
#include "Math/Math.hpp"
#include "Math/Trigonometric.hpp"

RM3508::RM3508(Connectivity &connectivity, uint16_t sendid, uint16_t receive_id,
               int8_t cw, float ratio)
    : Motor(connectivity, sendid, receive_id)
{
    this->clockwise *= cw;
    this->ratio = ratio;
}

RM3508::~RM3508()
{
}

RM3508 &RM3508::init()
{
    state.status = STATUS_INITUALIZING;

    state.position = 0;
    state.velocity = 0;
    state.toreque = 0;
    state.temprature = 0;

    refState.position = 0;
    refState.velocity = 0;
    refState.toreque = 0;
    refState.temprature = 0;

    // 如果有软限位选项，设置软限位，没有则初始化完成
    if (!(option & MOTOR_SOFT_ZERO)) {
        state.status = STATUS_INITUALIZED;
    }

    return *this;
}

RM3508 &RM3508::deInit()
{
    state.status = STATUS_DEINITUALIZING;

    state.position = 0;
    state.velocity = 0;
    state.toreque = 0;
    state.temprature = 0;

    refState.position = 0;
    refState.velocity = 0;
    refState.toreque = 0;
    refState.temprature = 0;

    state.status = STATUS_DEINITUALIZED;

    return *this;
}

RM3508 &RM3508::encodeControlMessage()
{
    auto current = clockwise * calculateControlData() * ifInitialed() / ratio *
                   ratio_0 / 0.3f;
    if (current > MAX_CURRENT)
        current = MAX_CURRENT;
    else if (current < -MAX_CURRENT)
        current = -MAX_CURRENT;
    int16_t data = current / MAX_CURRENT * MAX_CURRENT_DATA;
    if (connectivity.method == Connectivity::Method::CAN) {
        return encodeCanData(data);
    } else if (connectivity.method == Connectivity::Method::FDCAN) {
        return encodeFdcanData(data);
    }

    return *this;
}

RM3508 &RM3508::decodeFeedbackMessage()
{
    uint8_t *data = nullptr;
    if (connectivity.method == Connectivity::Method::CAN) {
        data = getCanData();
    } else if (connectivity.method == Connectivity::Method::FDCAN) {
        data = getFdcanData();
    }

    auto position =
        1.0f * clockwise * (data[0] << 8 | data[1]) / MAX_POISION_DATA * 2 * PI;
    auto velocity = 1.0f * clockwise * (int16_t)(data[2] << 8 | data[3]);
    auto toreque = 1.0f * clockwise * (int16_t)(data[4] << 8 | data[5]) /
                   MAX_CURRENT_DATA * MAX_CURRENT * 0.3f / ratio_0;
    state.temprature = data[6];
    if (last_position != 9999.0f) {
        if (position - last_position > 2.0f)
            count--;
        else if (position - last_position < -2.0f)
            count++;
        if (count > ratio / 2)
            count = -ratio / 2;
        else if (count < -ratio / 2)
            count = ratio / 2;
    }
    last_position = position;
    // TODO: 软限位和软零点，参考 A1 电机
    state.position =
        (position + count * 2 * PI) / ratio; // 位置和速度都除以齿轮比
    state.velocity = velocity / ratio;
    state.toreque = toreque * ratio; // 力矩乘以齿轮比

    return *this;
}

float RM3508::calculateControlData()
{
    // TODO: 软限位和软零点，参考 A1 电机
    // 位置过零点处理
    if (getTargetState().position - getState().position > 180.0f)
        getTargetState().position -= 360.0f;
    else if (getTargetState().position - getState().position < -180.0f)
        getTargetState().position += 360.0f;
    // 计算控制量
    refState.position = getTargetState().position;
    refState.velocity = getTargetState().velocity;
    refState.toreque = getTargetState().toreque;
    if (angleLoop != nullptr) {
        refState.velocity +=
            angleLoop->calculate(refState.position, state.position);
    }
    if (speedLoop != nullptr) {
        refState.toreque +=
            speedLoop->calculate(refState.velocity, state.velocity);
    }

    return refState.toreque;
}

RM3508 &RM3508::encodeCanData(uint16_t data)
{
#ifdef __CAN_H__
    /* 设置 CAN 标准帧标识符 */
    /* 若 id 为 1 ～ 4, 标识符 0x200 */
    /* 若 id 为 5 ～ 8, 标识符 0x1FF */
    uint16_t index = send_id;
    CAN::xTransmissionFrame_t *sendFrame =
        (CAN::xTransmissionFrame_t *)connectivity.getSendFrame();
    if (index < 5) {
        sendFrame->header.StdId = 0x200;
    } else {
        sendFrame->header.StdId = 0x1ff;
        index -= 4; // 5 -> 1, 6 -> 2, 7 -> 3, 8 -> 4
    }
    /* 帧类型：标准帧 */
    sendFrame->header.ExtId = 0;
    sendFrame->header.IDE = CAN_ID_STD;
    sendFrame->header.RTR = CAN_RTR_DATA;
    /* DLC 8 字节 */
    sendFrame->header.DLC = 8;

    /* 帧格式 Data */
    /* 高 8 位在前，低 8 位在后 */
    sendFrame->data[(index - 1) * 2] = data >> 8;
    sendFrame->data[(index - 1) * 2 + 1] = data;
#endif
    return *this;
}

uint8_t *RM3508::getCanData()
{
#ifdef __CAN_H__
    return ((CAN::xReceptionFrame_t *)(connectivity.getReceiveFrame()))->data;
#endif
    return nullptr;
}

RM3508 &RM3508::encodeFdcanData(uint16_t data)
{
#ifdef __FDCAN_H__

    uint16_t index = send_id;
    FDCAN::xTransmissionFrame_t *sendFrame =
        (FDCAN::xTransmissionFrame_t *)connectivity.getSendFrame();
    if (index < 5) {
        sendFrame->header.Identifier = 0x200;
    } else {
        sendFrame->header.Identifier = 0x1ff;
        index -= 4;
    }

    /* 帧类型：标准帧 */
    sendFrame->header.IdType = FDCAN_STANDARD_ID;
    sendFrame->header.DataLength = 8;

    /* 高 8 位在前，低 8 位在后 */
    sendFrame->data[(index - 1) * 2] = data >> 8;
    sendFrame->data[(index - 1) * 2 + 1] = data;

#endif
    return *this;
}

uint8_t *RM3508::getFdcanData()
{
#ifdef __FDCAN_H__
    return ((FDCAN::xReceptionFrame_t *)(connectivity.getReceiveFrame()))->data;
#endif
    return nullptr;
}