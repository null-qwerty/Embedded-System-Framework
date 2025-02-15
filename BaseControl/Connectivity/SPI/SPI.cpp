#include "BaseControl/Connectivity/SPI/SPI.hpp"
#include "stm32f4xx_hal_spi.h"

SPI::SPI(SPI_HandleTypeDef *hspi, dmaOption dma)
    : hspi(hspi)
    , dma(dma)
{
}

SPI::~SPI()
{
}

SPI &SPI::init()
{
    HAL_SPI_DeInit(hspi);
    HAL_SPI_Init(hspi);
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
    if (dma & dmaOption::TX) {
        if (hspi->hdmatx->State == HAL_DMA_STATE_READY) {
            HAL_SPI_Transmit_DMA(hspi, xSendFrame.data, xSendFrame.length);
            while (hspi->hdmatx->State != HAL_DMA_STATE_READY)
                ; // 等待发送完成
            return HAL_OK;
        }
        return HAL_BUSY;
    }
    return HAL_SPI_Transmit(hspi, xSendFrame.data, xSendFrame.length,
                            HAL_MAX_DELAY);
}

uint8_t SPI::receiveMessage()
{
    if (dma & dmaOption::RX) {
        if (hspi->hdmarx->State == HAL_DMA_STATE_READY) {
            HAL_SPI_Receive_DMA(hspi, xReceiveFrame.data, xReceiveFrame.length);
            while (hspi->hdmarx->State != HAL_DMA_STATE_READY)
                ; // 等待接收完成
            return HAL_OK;
        }
        return HAL_BUSY;
    }
    return HAL_SPI_Receive(hspi, xReceiveFrame.data, xReceiveFrame.length,
                           HAL_MAX_DELAY);
}

uint8_t SPI::sendReceiveMessage()
{
    if (dma == dmaOption::RX_TX) {
        if (hspi->hdmarx->State == HAL_DMA_STATE_READY &&
            hspi->hdmatx->State == HAL_DMA_STATE_READY) {
            HAL_SPI_TransmitReceive_DMA(hspi, xSendFrame.data,
                                        xReceiveFrame.data,
                                        xReceiveFrame.length);
            while (hspi->hdmarx->State != HAL_DMA_STATE_READY ||
                   hspi->hdmatx->State != HAL_DMA_STATE_READY)
                ; // 等待发送接收完成
            return HAL_OK;
        }
        return HAL_BUSY;
    }
    return HAL_SPI_TransmitReceive(hspi, xSendFrame.data, xReceiveFrame.data,
                                   xReceiveFrame.length, HAL_MAX_DELAY);
}