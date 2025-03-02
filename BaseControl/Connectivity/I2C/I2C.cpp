#include "BaseControl/Connectivity/I2C/I2C.hpp"
#include <stdint.h>

#ifdef __I2C_H__
I2C::I2C(I2C_HandleTypeDef *hi2c, dmaOption dma, bool isMaster)
    : hi2c(hi2c)
    , isMaster(isMaster)
    , dma(dma)
{
    this->method = Method::I2C;
}

I2C::~I2C()
{
}

I2C &I2C::init()
{
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
    return *this;
}

void *I2C::getReceiveFrame()
{
    return &xReceiveFrame;
}

void *I2C::getSendFrame()
{
    return &xSendFrame;
}

uint8_t I2C::getState()
{
    return HAL_I2C_GetState(hi2c);
}

uint8_t I2C::sendMessage()
{
    if (hi2c->State == HAL_I2C_STATE_BUSY_TX)
        return HAL_BUSY;

    if (xSendFrame.regAddress != 0xff)
        return writeMemory();

    if (dma & TX) {
        if (hi2c->hdmatx->State == HAL_DMA_STATE_BUSY)
            return HAL_BUSY;
        if (isMaster)
            HAL_I2C_Master_Transmit_DMA(hi2c, xSendFrame.devAddress,
                                        xSendFrame.data, xSendFrame.lenth);
        else
            HAL_I2C_Slave_Transmit_DMA(hi2c, xSendFrame.data, xSendFrame.lenth);
        return HAL_OK;
    }

    if (isMaster)
        return HAL_I2C_Master_Transmit(hi2c, xSendFrame.devAddress,
                                       xSendFrame.data, xSendFrame.lenth, 1000);
    else
        return HAL_I2C_Slave_Transmit(hi2c, xSendFrame.data, xSendFrame.lenth,
                                      1000);
}

uint8_t I2C::receiveMessage()
{
    if (hi2c->State == HAL_I2C_STATE_BUSY_RX)
        return HAL_BUSY;
    if (xReceiveFrame.regAddress != 0xff)
        return readMemory();

    if (dma & RX) {
        if (hi2c->hdmarx->State == HAL_DMA_STATE_BUSY)
            return HAL_BUSY;
        if (isMaster)
            HAL_I2C_Master_Receive_DMA(hi2c, xReceiveFrame.devAddress,
                                       xReceiveFrame.data, xReceiveFrame.lenth);
        else
            HAL_I2C_Slave_Receive_DMA(hi2c, xReceiveFrame.data,
                                      xReceiveFrame.lenth);
        return HAL_OK;
    }
    if (isMaster)
        return HAL_I2C_Master_Receive(hi2c, xReceiveFrame.devAddress,
                                      xReceiveFrame.data, xReceiveFrame.lenth,
                                      1000);
    else
        return HAL_I2C_Slave_Receive(hi2c, xReceiveFrame.data,
                                     xReceiveFrame.lenth, 1000);
}

uint8_t I2C::readMemory()
{
    if (dma & RX) {
        if (hi2c->hdmarx->State == HAL_DMA_STATE_BUSY)
            return HAL_BUSY;
        HAL_I2C_Mem_Read_DMA(hi2c, xReceiveFrame.devAddress << 1,
                             xReceiveFrame.regAddress, I2C_MEMADD_SIZE_8BIT,
                             xReceiveFrame.data, xReceiveFrame.lenth);
        return HAL_OK;
    }
    return HAL_I2C_Mem_Read(hi2c, xReceiveFrame.devAddress << 1,
                            xReceiveFrame.regAddress, I2C_MEMADD_SIZE_8BIT,
                            xReceiveFrame.data, xReceiveFrame.lenth, 1000);
}

uint8_t I2C::writeMemory()
{
    if (dma & TX) {
        if (hi2c->hdmatx->State == HAL_DMA_STATE_BUSY)
            return HAL_BUSY;
        HAL_I2C_Mem_Write_DMA(hi2c, xSendFrame.devAddress << 1,
                              xSendFrame.regAddress, I2C_MEMADD_SIZE_8BIT,
                              xSendFrame.data, xSendFrame.lenth);
        return HAL_OK;
    }
    return HAL_I2C_Mem_Write(hi2c, xSendFrame.devAddress << 1,
                             xSendFrame.regAddress, I2C_MEMADD_SIZE_8BIT,
                             xSendFrame.data, xSendFrame.lenth, 1000);
}

uint8_t I2C::sendReceiveMessage()
{
    return sendMessage() << 4 | receiveMessage();
}

I2C &I2C::operator=(const I2C &other)
{
    hi2c = other.hi2c;
    isMaster = other.isMaster;
    return *this;
}
#endif