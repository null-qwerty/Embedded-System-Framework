#include "BaseControl/Connectivity/UART/UART.hpp"

UART::UART(UART_HandleTypeDef *huart)
    : huart(huart)
{
}

UART::~UART(void)
{
}

UART &UART::init()
{
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);
    return *this;
}

void *UART::getReceiveFrame()
{
    return &xReceiveFrame;
}

void *UART::getSendFrame()
{
    return &xSendFrame;
}

uint8_t UART::getState()
{
    return HAL_UART_GetState(huart);
}

uint8_t UART::sendMessage()
{
    return HAL_UART_Transmit(huart, xSendFrame.data, xSendFrame.length, 20);
}

uint8_t UART::receiveMessage()
{
    return HAL_UART_Receive(huart, xReceiveFrame.data, xReceiveFrame.length,
                            20);
}

uint8_t UART::sendReceiveMessage()
{
    return sendMessage() << 4 | receiveMessage();
}