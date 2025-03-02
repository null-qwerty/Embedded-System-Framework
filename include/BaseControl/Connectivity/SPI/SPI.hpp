#pragma once

#include "main.h"

#include "BaseControl/Connectivity/Connectivity.hpp"

#ifdef __SPI_H__

class SPI : public Connectivity {
public:
    enum dmaOption { DISABLE = 0, RX = 1, TX = 2, RX_TX = 3 };
    typedef struct xSPIFrame_s {
        uint8_t *data = nullptr;
        uint16_t length = 0;
    } xSPIFrame_t;

    SPI(SPI_HandleTypeDef *hspi, dmaOption dma = dmaOption::DISABLE);
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

    dmaOption dma = dmaOption::DISABLE;
};
#endif