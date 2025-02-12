#pragma once

#include "BaseControl/Connectivity/Connectivity.hpp"

class UART : public Connectivity {
public:
    typedef struct xUARTFrame_s {
        uint8_t *data;
        uint16_t length;
    } xUARTFrame_t;
    /**
     * @brief UART 构造函数
     *
     * @param huart UART 句柄
     */
    UART(UART_HandleTypeDef *huart);
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
};