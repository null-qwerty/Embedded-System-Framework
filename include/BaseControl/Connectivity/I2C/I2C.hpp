#pragma once

#include "i2c.h"

#include "BaseControl/Connectivity/Connectivity.hpp"

class I2C : public Connectivity {
public:
    enum dmaOption { DISABLE = 0, RX = 1, TX = 2, RX_TX = 3 };
    typedef struct xI2CFrame_s {
        uint8_t devAddress;
        uint8_t regAddress = 0xff;
        uint8_t *data;
        uint16_t lenth;
    } xI2CFrame_t;

    I2C(I2C_HandleTypeDef *hi2c, dmaOption dma = dmaOption::DISABLE,
        bool isMaster = true);
    ~I2C();

    virtual I2C &init() override;
    virtual void *getReceiveFrame() override;
    virtual void *getSendFrame() override;
    virtual uint8_t getState() override;
    virtual uint8_t sendMessage() override;
    virtual uint8_t receiveMessage() override;
    virtual uint8_t sendReceiveMessage() override;

    I2C &operator=(const I2C &other);

private:
    I2C_HandleTypeDef *hi2c = nullptr;

    xI2CFrame_t xReceiveFrame = {};
    xI2CFrame_t xSendFrame = {};

    bool isMaster = true;

    dmaOption dma = dmaOption::DISABLE;

    uint8_t readMemory();
    uint8_t writeMemory();
};