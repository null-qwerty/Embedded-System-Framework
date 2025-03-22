#include "BaseControl/Motor/UnitreeA1.hpp"
#include "Math/Math.hpp"
#include <string.h>

UnitreeA1::UnitreeA1(Connectivity &connectivity, uint16_t send_id,
                     uint16_t receive_id, int8_t cw)
    : Motor(connectivity, send_id, receive_id)
{
    this->clockwise *= cw;
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

    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->readIndex = 1;
    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->data[0] =
        (uint8_t *)(&sendBuffer[0]);
    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->data[1] =
        (uint8_t *)(&sendBuffer[1]);

    memset(sendBuffer, 0, 2 * sizeof(sendData));

    sendBuffer[0].header.start = 0xeefe;
    sendBuffer[0].header.id = send_id;
    sendBuffer[0].data.mode = 10;
    sendBuffer[0].crc = crc32_core((uint32_t *)(sendBuffer), 7);

    ifEnable = 1;

    return *this;
}

UnitreeA1 &UnitreeA1::deInit()
{
    ifEnable = 0;

    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->readIndex = 1;
    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->data[0] =
        (uint8_t *)(&sendBuffer[0]);
    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->data[1] =
        (uint8_t *)(&sendBuffer[1]);

    memset(sendBuffer, 0, 2 * sizeof(sendData));

    sendBuffer[0].header.start = 0xeefe;
    sendBuffer[0].header.id = send_id;
    sendBuffer[0].data.mode = 0;
    sendBuffer[0].crc = crc32_core((uint32_t *)(sendBuffer), 7);

    return *this;
}

UnitreeA1 &UnitreeA1::encodeControlMessage()
{
    int16_t data = calculateControlData();

    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->readIndex =
        sendReadIndex;
    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->data[0] =
        (uint8_t *)(&sendBuffer[0]);
    ((UART::xUARTFrame_t *)(connectivity.getSendFrame()))->data[1] =
        (uint8_t *)(&sendBuffer[1]);

    memset(&sendBuffer[sendReadIndex], 0, sizeof(sendData));

    sendBuffer[sendReadIndex].header.start = 0xeefe;
    sendBuffer[sendReadIndex].header.id = send_id;
    sendBuffer[sendReadIndex].data.mode = 10;
    sendBuffer[sendReadIndex].data.T = data * 256;
    sendBuffer[sendReadIndex].crc = crc32_core((uint32_t *)(sendBuffer), 7);

    sendReadIndex = 1 - sendReadIndex;

    ((UART::xUARTFrame_t *)(connectivity.getReceiveFrame()))->readIndex =
        receiveReadIndex;
    ((UART::xUARTFrame_t *)(connectivity.getReceiveFrame()))->data[0] =
        (uint8_t *)(&receiveBuffer[0]);
    ((UART::xUARTFrame_t *)(connectivity.getReceiveFrame()))->data[1] =
        (uint8_t *)(&receiveBuffer[1]);

    return *this;
}

UnitreeA1 &UnitreeA1::decodeFeedbackMessage()
{
    if (receiveBuffer[receiveReadIndex].header.start == 0xeefe &&
        receiveBuffer[receiveReadIndex].header.id == receive_id &&
        receiveBuffer[receiveReadIndex].crc ==
            crc32_core((uint32_t *)(receiveBuffer), 18)) {
        state.position =
            receiveBuffer[receiveReadIndex].data.Pos * 2 * PI / 16384.;
        state.velocity = receiveBuffer[receiveReadIndex].data.W / 128.;
        state.toreque = receiveBuffer[receiveReadIndex].data.T / 256.;
        state.temprature = receiveBuffer[receiveReadIndex].data.Temp;
    }

    receiveReadIndex = 1 - receiveReadIndex;

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
