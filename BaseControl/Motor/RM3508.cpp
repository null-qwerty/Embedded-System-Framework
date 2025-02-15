#include "BaseControl/Motor/RM3508.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"
#include "BaseControl/Motor/Motor.hpp"
#include "stm32f4xx_hal_can.h"

RM3508::RM3508(CAN &can, uint16_t send_id, uint16_t receive_id, int8_t cw)
    : Motor(can, send_id, receive_id)
{
    this->clockwise *= cw;
}

RM3508::~RM3508()
{
}

RM3508 &RM3508::init()
{
    state.position = 0;
    state.velocity = 0;
    state.toreque = 0;
    state.temprature = 0;

    refState.position = 0;
    refState.velocity = 0;
    refState.toreque = 0;
    refState.temprature = 0;

    initialed = true;

    return *this;
}

RM3508 &RM3508::deInit()
{
    state.position = 0;
    state.velocity = 0;
    state.toreque = 0;
    state.temprature = 0;

    refState.position = 0;
    refState.velocity = 0;
    refState.toreque = 0;
    refState.temprature = 0;

    initialed = false;

    return *this;
}

RM3508 &RM3508::encodeControlMessage()
{
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
    int16_t data = clockwise * calculateControlData() * ifInitialed();
    /* 高 8 位在前，低 8 位在后 */
    sendFrame->data[(index - 1) * 2] = data >> 8;
    sendFrame->data[(index - 1) * 2 + 1] = data;

    return *this;
}

RM3508 &RM3508::decodeFeedbackMessage()
{
    uint8_t *data =
        ((CAN::xReceptionFrame_t *)(connectivity.getReceiveFrame()))->data;

    state.position =
        1.0 * clockwise * (data[0] << 8 | data[1]) / MAX_POISION_DATA * 360.0f;
    state.velocity = 1.0 * clockwise * (int16_t)(data[2] << 8 | data[3]);
    state.toreque = 1.0 * clockwise * (int16_t)(data[4] << 8 | data[5]);
    state.temprature = data[6];

    return *this;
}

float RM3508::calculateControlData()
{
    // 位置过零点处理
    if (getTargetState().position - getState().position > 180.0f)
        getTargetState().position -= 360.0f;
    else if (getTargetState().position - getState().position < -180.0f)
        getTargetState().position += 360.0f;
    // 计算控制量
    refState.position = getTargetState().position;
    if (angleLoop != nullptr) {
        refState.velocity =
            getTargetState().velocity +
            angleLoop->calculate(refState.position, state.position);
    } else {
        refState.velocity = getTargetState().velocity;
    }
    if (speedLoop != nullptr) {
        refState.toreque =
            getTargetState().toreque +
            speedLoop->calculate(refState.velocity, state.velocity);
    } else {
        refState.toreque = getTargetState().toreque;
    }

    return refState.toreque;
}