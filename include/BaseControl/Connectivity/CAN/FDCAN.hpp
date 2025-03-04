#pragma once

#include "main.h"

#include "BaseControl/Connectivity/Connectivity.hpp"

#ifdef __FDCAN_H__

class FDCAN : public Connectivity {
public:
    typedef struct xReceptionFrame_s {
        FDCAN_RxHeaderTypeDef header;
        uint8_t data[8];
    } xReceptionFrame_t;
    typedef struct xTransmissionFrame_s {
        FDCAN_TxHeaderTypeDef header;
        uint8_t data[8];
    } xTransmissionFrame_t;

    FDCAN(FDCAN_HandleTypeDef *hcan, FDCAN_FilterTypeDef &filter,
          uint32_t fdcan_buffer_index);
    ~FDCAN();

    virtual FDCAN &init() override;
    virtual void *getSendFrame() override;
    virtual void *getReceiveFrame() override;
    virtual uint8_t getState() override;
    virtual uint8_t sendMessage() override;
    virtual uint8_t receiveMessage() override;
    virtual uint8_t sendReceiveMessage() override;

    FDCAN_FilterTypeDef &getFilter();
    /**
     * @brief 重载赋值运算符为浅拷贝
     *
     * @param other 赋值对象
     * @return FDCAN& 返回自身引用
     */
    FDCAN &operator=(const FDCAN &other);

private:
    FDCAN_HandleTypeDef *hcan = nullptr;
    FDCAN_FilterTypeDef filter = {};
    uint32_t fdcan_buffer_index = 0;

    xReceptionFrame_t receiveFrame = {};
    xTransmissionFrame_t sendFrame = {};
};

#endif
