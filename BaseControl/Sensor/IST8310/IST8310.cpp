#include "BaseControl/Sensor/IST8310/IST8310.hpp"
#include "BaseControl/Sensor/IST8310/IST8310_reg.hpp"

#include "BaseControl/Connectivity/I2C/I2C.hpp"

#include "main.h"

IST8310::IST8310(Connectivity &connectivity, uint8_t address)
    : Sensor(connectivity)
    , address(address)
{
}

IST8310 &IST8310::init()
{
    connectivity.init();
    // 重启 IST8310
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin(RSTN_IST8310_GPIO_Port, RSTN_IST8310_Pin, GPIO_PIN_SET);
    HAL_Delay(50);

    writeByte(IST8310_CNTL2_REG, 0x00);
    HAL_Delay(150);
    writeByte(IST8310_AVGCNTL_REG, IST8310_SAMPLE_AVG_8);
    HAL_Delay(150);
    writeByte(IST8310_CNTL1_REG, IST8310_CONTINUOUS_MODE_200HZ);

    return *this;
}

void *IST8310::getData()
{
    if (readData(IST8310_DATA_X_L_REG, dataBuffer, 6) != HAL_OK)
        init();
    if (readData(IST8310_DATA_TEMP_L_REG, dataBuffer + 6, 2) != HAL_OK)
        init();

    data.x = *((int16_t *)(dataBuffer + 0)) * 0.3; // 单位：uT
    data.y = *((int16_t *)(dataBuffer + 2)) * 0.3;
    data.z = *((int16_t *)(dataBuffer + 4)) * 0.3;
    data.temprature = *((int16_t *)(dataBuffer + 6)) / 65536.0 * (85 + 40) -
                      40; // 85℃为最大温度，-40℃为最小温度

    return &data;
}

IST8310 &IST8310::reset()
{
    init();
    return *this;
}

uint8_t IST8310::writeByte(uint8_t reg, uint8_t data)
{
    ((I2C::xI2CFrame_t *)connectivity.getSendFrame())->devAddress = address;
    ((I2C::xI2CFrame_t *)connectivity.getSendFrame())->regAddress = reg;
    ((I2C::xI2CFrame_t *)connectivity.getSendFrame())->data = &data;
    ((I2C::xI2CFrame_t *)connectivity.getSendFrame())->lenth = 1;

    return connectivity.sendMessage();
}

uint8_t IST8310::readByte(uint8_t reg, uint8_t *data)
{
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->devAddress = address;
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->regAddress = reg;
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->data = data;
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->lenth = 1;

    return connectivity.receiveMessage();
}

uint8_t IST8310::readData(uint8_t reg, uint8_t *data, uint8_t lenth)
{
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->devAddress = address;
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->regAddress = reg;
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->data = data;
    ((I2C::xI2CFrame_t *)connectivity.getReceiveFrame())->lenth = lenth;

    return connectivity.receiveMessage();
}