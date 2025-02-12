#pragma once

#include "spi.h"

#include "BaseControl/Connectivity/Connectivity.hpp"

class SPI : public Connectivity {
public:
    typedef struct xSPIFrame_s {
        uint8_t *data = nullptr;
        uint16_t length = 0;
    } xSPIFrame_t;

    SPI(SPI_HandleTypeDef *hspi);
    ~SPI();

    virtual SPI &init() override;
    virtual void *getReceiveFrame() override;
    virtual void *getSendFrame() override;
    virtual uint8_t getState() override;
    virtual uint8_t sendMessage() override;
    virtual uint8_t receiveMessage() override;
    virtual uint8_t sendReceiveMessage() override;

    SPI &operator=(const SPI &other);

private:
    SPI_HandleTypeDef *hspi = nullptr;

    xSPIFrame_t xReceiveFrame = {};
    xSPIFrame_t xSendFrame = {};
};