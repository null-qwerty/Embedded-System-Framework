#include "BaseControl/Motor/UnitreeA1.hpp"
#include "Math/Math.hpp"

UnitreeA1::UnitreeA1(Connectivity &connectivity, uint16_t send_id,
                     uint16_t receive_id, int8_t cw, float ratio)
    : Motor(connectivity, send_id, receive_id)
{
    this->clockwise *= cw;
    this->ratio = ratio;
}

UnitreeA1::~UnitreeA1()
{
}

UnitreeA1 &UnitreeA1::init()
{
    refState.position = 0;
    refState.velocity = 0;
    refState.toreque = 0;
    refState.temprature = 0;

    state.position = 0;
    state.velocity = 0;
    state.toreque = 0;
    state.temprature = 0;

    auto sendframe = (UART::xUARTFrame_t *)(connectivity.getSendFrame());
    auto readindex = sendframe->readIndex;
    auto sendBuffer = (sendData *)(sendframe->data[readindex]);

    sendBuffer->header.start = 0xeefe;
    sendBuffer->header.id = send_id;
    sendBuffer->data.mode = 10;
    sendBuffer->crc = crc32_core((uint32_t *)(sendBuffer), 7);

    ifEnable = 1;

    return *this;
}

UnitreeA1 &UnitreeA1::deInit()
{
    ifEnable = 0;

    auto sendframe = (UART::xUARTFrame_t *)(connectivity.getSendFrame());
    auto readindex = sendframe->readIndex;
    auto sendBuffer = (sendData *)(sendframe->data[readindex]);

    sendBuffer->header.start = 0xeefe;
    sendBuffer->header.id = send_id;
    sendBuffer->data.mode = 10;
    sendBuffer->crc = crc32_core((uint32_t *)(sendBuffer), 7);

    return *this;
}

UnitreeA1 &UnitreeA1::encodeControlMessage()
{
    int16_t data = clockwise * calculateControlData() / ratio;

    auto sendframe = (UART::xUARTFrame_t *)(connectivity.getSendFrame());
    auto readindex = sendframe->readIndex;
    auto sendBuffer = (sendData *)(sendframe->data[readindex]);

    sendBuffer->header.start = 0xeefe;
    sendBuffer->header.id = send_id;
    sendBuffer->data.mode = 10;
    sendBuffer->data.T = data * 256;
    sendBuffer->crc = crc32_core((uint32_t *)(sendBuffer), 7);

    return *this;
}

UnitreeA1 &UnitreeA1::decodeFeedbackMessage()
{
    auto receiveframe = (UART::xUARTFrame_t *)(connectivity.getReceiveFrame());
    auto readindex = receiveframe->readIndex;
    auto receiveBuffer = (receiveData *)(receiveframe->data[readindex]);
    if (receiveBuffer->header.start == 0xeefe &&
        receiveBuffer->header.id == receive_id &&
        receiveBuffer->crc == crc32_core((uint32_t *)(receiveBuffer), 18)) {
        state.position =
            1.0 * clockwise * receiveBuffer->data.Pos * 2 * PI / 16384.;
        state.velocity = 1.0 * clockwise * receiveBuffer->data.W / 128.;
        state.toreque = 1.0 * clockwise * receiveBuffer->data.T / 256.;
        state.temprature = receiveBuffer->data.Temp;

        // 计算输出轴位置/速度/力矩
        state.position /= ratio; // 位置和速度都除以齿轮比
        state.velocity /= ratio;
        state.toreque *= ratio; // 力矩乘以齿轮比
    }

    return *this;
}

float UnitreeA1::calculateControlData()
{
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

uint32_t UnitreeA1::crc32_core(uint32_t *ptr, uint32_t len)
{
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}
