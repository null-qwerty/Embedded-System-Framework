#include "BaseControl/Motor/DM4310.hpp"
#include "BaseControl/Connectivity/CAN/CAN.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"

#include "Math/Math.hpp"

DM4310::DM4310(CAN &can, uint16_t send_id, uint16_t receive_id, int8_t cw)
    : Motor(can, send_id, receive_id)
{
    this->clockwise *= cw;
}

DM4310::~DM4310()
{
}

DM4310 &DM4310::init()
{
    refState.position = 0;
    refState.velocity = 0;
    refState.toreque = 0;
    refState.temprature = 0;

    state.position = 0;
    state.velocity = 0;
    state.toreque = 0;
    state.temprature = 0;

    /* 设置 CAN 标准帧标识符，报文为 CAN_ID */
    CAN::xTransmissionFrame_t *sendFrame =
        (CAN::xTransmissionFrame_t *)connectivity.getSendFrame();
    sendFrame->header.StdId = this->send_id;
    /* 帧类型：标准帧 */
    sendFrame->header.ExtId = 0;
    sendFrame->header.IDE = CAN_ID_STD;
    sendFrame->header.RTR = CAN_RTR_DATA;
    /* DLC 8 字节 */
    sendFrame->header.DLC = 8;
    // 使能报文
    sendFrame->data[0] = 0xff;
    sendFrame->data[1] = 0xff;
    sendFrame->data[2] = 0xff;
    sendFrame->data[3] = 0xff;
    sendFrame->data[4] = 0xff;
    sendFrame->data[5] = 0xff;
    sendFrame->data[6] = 0xff;
    sendFrame->data[7] = 0xfc;

    connectivity.sendMessage();

    return *this;
}

DM4310 &DM4310::deInit()
{
    /* 设置 CAN 标准帧标识符，报文为 CAN_ID */
    CAN::xTransmissionFrame_t *sendFrame =
        (CAN::xTransmissionFrame_t *)connectivity.getSendFrame();
    sendFrame->header.StdId = this->send_id;
    /* 帧类型：标准帧 */
    sendFrame->header.ExtId = 0;
    sendFrame->header.IDE = CAN_ID_STD;
    sendFrame->header.RTR = CAN_RTR_DATA;
    /* DLC 8 字节 */
    sendFrame->header.DLC = 8;
    // 失能报文
    sendFrame->data[0] = 0xff;
    sendFrame->data[1] = 0xff;
    sendFrame->data[2] = 0xff;
    sendFrame->data[3] = 0xff;
    sendFrame->data[4] = 0xff;
    sendFrame->data[5] = 0xff;
    sendFrame->data[6] = 0xff;
    sendFrame->data[7] = 0xfd;

    connectivity.sendMessage();

    return *this;
}

DM4310 &DM4310::encodeControlMessage()
{
    /* 设置 CAN 标准帧标识符，报文为 CAN_ID */
    CAN::xTransmissionFrame_t *sendFrame =
        (CAN::xTransmissionFrame_t *)connectivity.getSendFrame();
    sendFrame->header.StdId = this->send_id;
    /* 帧类型：标准帧 */
    sendFrame->header.ExtId = 0;
    sendFrame->header.IDE = CAN_ID_STD;
    sendFrame->header.RTR = CAN_RTR_DATA;
    /* DLC 8 字节 */
    sendFrame->header.DLC = 8;

    uint16_t data = linearFloat2Uint(clockwise * calculateControlData(),
                                     DM4310_MAX_TAU, -DM4310_MAX_TAU, 12);

    sendFrame->data[0] = 0x0000;
    sendFrame->data[1] = 0x0000;
    sendFrame->data[2] = 0x0000;
    sendFrame->data[3] = 0x0000;
    sendFrame->data[4] = 0x0000;
    sendFrame->data[5] = 0x0000;
    sendFrame->data[6] = 0x0000 | (data >> 8);
    sendFrame->data[7] = 0x0000 | (data & 0x00ff);

    return *this;
}

DM4310 &DM4310::decodeFeedbackMessage()
{
    uint8_t *data =
        ((CAN::xReceptionFrame_t *)(connectivity.getReceiveFrame()))->data;
    auto temp_p =
        clockwise *
        linearUint2Float((((uint16_t)data[1] << 8) | data[2]), PI, -PI, 16);
    auto temp_v =
        clockwise * linearUint2Float(((uint16_t)data[3] << 4) | (data[4] >> 4),
                                     DM4310_MAX_VEL, -DM4310_MAX_VEL, 12);
    auto temp_t = clockwise *
                  linearUint2Float((((uint16_t)data[4] & 0x0f) << 8) | data[5],
                                   DM4310_MAX_TAU, -DM4310_MAX_TAU, 12);
    state.position = state.position * 0.2 + temp_p * 0.8;
    state.velocity = state.velocity * 0.2 + temp_v * 0.8;
    state.toreque = state.toreque * 0.2 + temp_t * 0.8;

    state.temprature = data[7];

    return *this;
}

uint16_t DM4310::linearFloat2Uint(float x, float x_max, float x_min,
                                  uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float DM4310::linearUint2Float(uint16_t x, float x_max, float x_min,
                               uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x) * span / ((float)((1 << bits) - 1)) + offset;
}

float DM4310::calculateControlData()
{
    // 位置过零点处理
    if (getTargetState().position - state.position > 180.0f)
        getTargetState().position -= 360.0f;
    else if (getTargetState().position - state.position < -180.0f)
        getTargetState().position += 360.0f;

    // TODO 设置限位，后续放到外部
    if (getTargetState().position < 0.02)
        getTargetState().position = 0.02;
    if (getTargetState().position > 1.2)
        getTargetState().position = 1.2;
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