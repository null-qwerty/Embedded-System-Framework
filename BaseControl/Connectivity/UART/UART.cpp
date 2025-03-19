#include "BaseControl/Connectivity/UART/UART.hpp"

#if defined __USART_H__ || defined __UART_H__

UART::UART(UART_HandleTypeDef *huart, dmaOption dma)
    : huart(huart)
    , dma(dma)
{
    this->method = Method::UART;
}

UART::~UART(void)
{
}

UART &UART::init()
{
    HAL_UART_DeInit(huart);
    HAL_UART_Init(huart);
    if (dma & RX) {
        HAL_UART_Receive_DMA(huart,
                             xReceiveFrame.data[1 - xReceiveFrame.readIndex],
                             xReceiveFrame.length);
    }
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
    if (dma & TX) {
        xSendFrame.readIndex = 1 - xSendFrame.readIndex;
        HAL_UART_Transmit_DMA(huart, xSendFrame.data[xSendFrame.readIndex],
                              xSendFrame.length);
        return HAL_OK;
    }
    xSendFrame.readIndex = 1 - xSendFrame.readIndex;
    return HAL_UART_Transmit(huart, xSendFrame.data[xSendFrame.readIndex],
                             xSendFrame.length, 20);
}

uint8_t UART::receiveMessage()
{
    if (dma & RX) {
        if (huart->hdmarx->State == HAL_DMA_STATE_READY) {
            xReceiveFrame.readIndex = 1 - xReceiveFrame.readIndex;
            // huart->hdmarx->State = HAL_DMA_STATE_BUSY;
            HAL_UART_Receive_DMA(
                huart, xReceiveFrame.data[1 - xReceiveFrame.readIndex],
                xReceiveFrame.length);
            return HAL_OK;
        }
        return HAL_BUSY;
    }
    xReceiveFrame.readIndex = 1 - xReceiveFrame.readIndex;
    return HAL_UART_Receive(huart, xReceiveFrame.data[xReceiveFrame.readIndex],
                            xReceiveFrame.length, 20);
}

uint8_t UART::sendReceiveMessage()
{
    return sendMessage() << 4 | receiveMessage();
}
#endif