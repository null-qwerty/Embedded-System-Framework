#include "BaseControl/Connectivity/I2C/I2C.hpp"
#include "stm32f4xx_hal_i2c.h"

I2C::I2C(I2C_HandleTypeDef *hi2c, bool isMaster)
    : hi2c(hi2c)
    , isMaster(isMaster)
{
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
    if (isMaster)
        return HAL_I2C_Master_Transmit(hi2c, xSendFrame.address,
                                       xSendFrame.data, xSendFrame.lenth, 1000);
    else
        return HAL_I2C_Slave_Transmit(hi2c, xSendFrame.data, xSendFrame.lenth,
                                      1000);
}

uint8_t I2C::receiveMessage()
{
    if (isMaster)
        return HAL_I2C_Master_Receive(hi2c, xReceiveFrame.address,
                                      xReceiveFrame.data, xReceiveFrame.lenth,
                                      1000);
    else
        return HAL_I2C_Slave_Receive(hi2c, xReceiveFrame.data,
                                     xReceiveFrame.lenth, 1000);
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