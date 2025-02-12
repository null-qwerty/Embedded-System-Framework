#pragma once

#include "i2c.h"

#include "BaseControl/Connectivity/Connectivity.hpp"

class I2C : public Connectivity {
public:
    typedef struct xI2CFrame_s {
        uint8_t address;
        uint8_t *data;
        uint16_t lenth;
    } xI2CFrame_t;

    I2C(I2C_HandleTypeDef *hi2c, bool isMaster = true);
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
};