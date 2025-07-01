#include "BaseControl/Motor/DM4310.hpp"

#include "Math/Math.hpp"

DM4310::DM4310(Connectivity &connectivity, uint16_t send_id,
               uint16_t receive_id, int8_t cw, float radio)
    : Motor(connectivity, send_id, receive_id)
{
    this->clockwise *= cw;
    this->radio = radio;
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

    uint8_t buffer[8];

    // 使能报文
    buffer[0] = 0xff;
    buffer[1] = 0xff;
    buffer[2] = 0xff;
    buffer[3] = 0xff;
    buffer[4] = 0xff;
    buffer[5] = 0xff;
    buffer[6] = 0xff;
    buffer[7] = 0xfc;

    if (connectivity.method == Connectivity::Method::CAN) {
        encodeCAN(buffer);
    } else if (connectivity.method == Connectivity::Method::FDCAN) {
        encodeFDCAN(buffer);
    }

    connectivity.sendMessage();

    return *this;
}

DM4310 &DM4310::deInit()
{
    uint8_t buffer[8];

    // 失能报文
    buffer[0] = 0xff;
    buffer[1] = 0xff;
    buffer[2] = 0xff;
    buffer[3] = 0xff;
    buffer[4] = 0xff;
    buffer[5] = 0xff;
    buffer[6] = 0xff;
    buffer[7] = 0xfd;

    if (connectivity.method == Connectivity::Method::CAN) {
        encodeCAN(buffer);
    } else if (connectivity.method == Connectivity::Method::FDCAN) {
        encodeFDCAN(buffer);
    }

    connectivity.sendMessage();

    return *this;
}

DM4310 &DM4310::encodeControlMessage()
{
    uint8_t buffer[8];
    uint16_t data = linearFloat2Uint(clockwise * calculateControlData() / radio,
                                     DM4310_MAX_TAU, -DM4310_MAX_TAU, 12);

    buffer[0] = 0x0000;
    buffer[1] = 0x0000;
    buffer[2] = 0x0000;
    buffer[3] = 0x0000;
    buffer[4] = 0x0000;
    buffer[5] = 0x0000;
    buffer[6] = 0x0000 | (data >> 8);
    buffer[7] = 0x0000 | (data & 0x00ff);

    if (connectivity.method == Connectivity::Method::CAN) {
        return encodeCAN(buffer);
    } else if (connectivity.method == Connectivity::Method::FDCAN) {
        return encodeFDCAN(buffer);
    }

    return *this;
}

DM4310 &DM4310::decodeFeedbackMessage()
{
    uint8_t *data = nullptr;
    if (connectivity.method == Connectivity::Method::CAN) {
        decodeCAN(data);
    } else if (connectivity.method == Connectivity::Method::FDCAN) {
        decodeFDCAN(data);
    }

    state.position =
        clockwise *
        linearUint2Float((((uint16_t)data[1] << 8) | data[2]), PI, -PI, 16);
    state.velocity =
        clockwise * linearUint2Float(((uint16_t)data[3] << 4) | (data[4] >> 4),
                                     DM4310_MAX_VEL, -DM4310_MAX_VEL, 12);
    state.toreque = clockwise * linearUint2Float(
                                    (((uint16_t)data[4] & 0x0f) << 8) | data[5],
                                    DM4310_MAX_TAU, -DM4310_MAX_TAU, 12);

    // 计算输出轴位置/速度/力矩
    state.position /= radio; // 位置和速度都除以齿轮比
    state.velocity /= radio;
    state.toreque *= radio; // 力矩乘以齿轮比

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

DM4310 &DM4310::encodeCAN(uint8_t *buffer)
{
#ifdef __CAN_H__
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

    sendFrame->data[0] = buffer[0];
    sendFrame->data[1] = buffer[1];
    sendFrame->data[2] = buffer[2];
    sendFrame->data[3] = buffer[3];
    sendFrame->data[4] = buffer[4];
    sendFrame->data[5] = buffer[5];
    sendFrame->data[6] = buffer[6];
    sendFrame->data[7] = buffer[7];
#endif

    return *this;
}

DM4310 &DM4310::decodeCAN(uint8_t *data)
{
#ifdef __CAN_H__
    data = ((CAN::xReceptionFrame_t *)(connectivity.getReceiveFrame()))->data;

#endif

    return *this;
}

DM4310 &DM4310::encodeFDCAN(uint8_t *buffer)
{
#ifdef __FDCAN_H__
    FDCAN::xTransmissionFrame_t *sendFrame =
        (FDCAN::xTransmissionFrame_t *)connectivity.getSendFrame();

    /* 帧类型：标准帧 */
    sendFrame->header.IdType = FDCAN_STANDARD_ID;
    sendFrame->header.DataLength = 8;

    sendFrame->data[0] = buffer[0];
    sendFrame->data[1] = buffer[1];
    sendFrame->data[2] = buffer[2];
    sendFrame->data[3] = buffer[3];
    sendFrame->data[4] = buffer[4];
    sendFrame->data[5] = buffer[5];
    sendFrame->data[6] = buffer[6];
    sendFrame->data[7] = buffer[7];

#endif
    return *this;
}

DM4310 &DM4310::decodeFDCAN(uint8_t *data)
{
#ifdef __FDCAN_H__
    data = ((FDCAN::xReceptionFrame_t *)(connectivity.getReceiveFrame()))->data;
#endif
    return *this;
}