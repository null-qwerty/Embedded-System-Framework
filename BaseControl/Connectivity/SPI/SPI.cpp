#include "BaseControl/Connectivity/SPI/SPI.hpp"

SPI::SPI(SPI_HandleTypeDef *hspi)
    : hspi(hspi)
{
}

SPI::~SPI()
{
}

SPI &SPI::init()
{
    return *this;
}

void *SPI::getReceiveFrame()
{
    return &xReceiveFrame;
}

void *SPI::getSendFrame()
{
    return &xSendFrame;
}

uint8_t SPI::getState()
{
    return HAL_SPI_GetState(hspi);
}

uint8_t SPI::sendMessage()
{
    return HAL_SPI_Transmit(hspi, xSendFrame.data, xSendFrame.length,
                            HAL_MAX_DELAY);
}

uint8_t SPI::receiveMessage()
{
    return HAL_SPI_Receive(hspi, xReceiveFrame.data, xReceiveFrame.length,
                           HAL_MAX_DELAY);
}

uint8_t SPI::sendReceiveMessage()
{
    return HAL_SPI_TransmitReceive(hspi, xSendFrame.data, xReceiveFrame.data,
                                   xReceiveFrame.length, HAL_MAX_DELAY);
}