#pragma once

#include "main.h"

#include "BaseControl/Connectivity/Connectivity.hpp"

#ifdef __CAN_H__
/**
 * @brief CAN 通信类，封装了 HAL 库的 CAN 相关函数
 *
 * @note 重载了赋值运算符为浅拷贝
 */
class CAN : public Connectivity {
public:
    typedef struct xReceptionFrame_s {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
    } xReceptionFrame_t;
    typedef struct xTransmissionFrame_s {
        CAN_TxHeaderTypeDef header;
        uint8_t data[8];
    } xTransmissionFrame_t;

    CAN(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef &filter);
    ~CAN();

    virtual CAN &init() override;
    virtual void *getSendFrame() override;
    virtual void *getReceiveFrame() override;
    virtual uint8_t getState() override;
    virtual uint8_t sendMessage() override;
    virtual uint8_t receiveMessage() override;
    virtual uint8_t sendReceiveMessage() override;

    CAN_FilterTypeDef &getFilter();
    /**
     * @brief 重载赋值运算符为浅拷贝
     *
     * @param other 赋值对象
     * @return CAN& 返回自身引用
     */
    CAN &operator=(const CAN &other);

private:
    CAN_HandleTypeDef *hcan = nullptr;
    uint32_t can_tx_mailbox = 0;
    CAN_FilterTypeDef filter = {};

    xReceptionFrame_t receiveFrame = {};
    xTransmissionFrame_t sendFrame = {};
};
#endif