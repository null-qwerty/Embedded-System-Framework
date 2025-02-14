#pragma once

#include "BaseControl/Connectivity/Connectivity.hpp"
#include "stm32f4xx.h"

class UART : public Connectivity {
public:
    enum dmaOption { DISABLE = 0, RX = 1, TX = 2, RX_TX = 3 };
    typedef struct xUARTFrame_s {
        uint8_t *data[2];
        uint16_t length;
        uint8_t readIndex = 1;
    } xUARTFrame_t;
    /**
     * @brief UART 构造函数
     *
     * @param huart UART 句柄
     * @param dma DMA 选项, 默认为 DISABLE
     */
    UART(UART_HandleTypeDef *huart, dmaOption dma = DISABLE);
    /**
     * @brief UART 析构函数
     *
     */
    ~UART(void);

    virtual UART &init() override;
    virtual void *getReceiveFrame() override;
    virtual void *getSendFrame() override;
    virtual uint8_t getState() override;
    virtual uint8_t sendMessage() override;
    virtual uint8_t receiveMessage() override;
    virtual uint8_t sendReceiveMessage() override;

private:
    UART_HandleTypeDef *huart = nullptr;
    xUARTFrame_t xReceiveFrame, xSendFrame;

    dmaOption dma = DISABLE;
};