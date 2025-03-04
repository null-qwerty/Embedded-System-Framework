#include "BaseControl/Connectivity/CAN/FDCAN.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"
#include "stm32h7xx_hal_fdcan.h"

#ifdef __FDCAN_H__

FDCAN::FDCAN(FDCAN_HandleTypeDef *hcan, FDCAN_FilterTypeDef &filter,
             uint32_t fdcan_buffer_index)
    : Connectivity()
    , hcan(hcan)
    , filter(filter)
    , fdcan_buffer_index(fdcan_buffer_index)
{
    this->method = Method::FDCAN;
}

FDCAN::~FDCAN()
{
}

FDCAN &FDCAN::init()
{
    // clang-format off
    while (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
        ;
    while (HAL_FDCAN_ConfigGlobalFilter(hcan, DISABLE, DISABLE,
                                        DISABLE, DISABLE))
        ;
    while (HAL_FDCAN_Start(hcan) != HAL_OK)
        ;
    while (HAL_FDCAN_ActivateNotification(hcan,
                                          filter.FilterConfig ==
                                                  FDCAN_FILTER_TO_RXFIFO0 ?
                                              FDCAN_IT_RX_FIFO0_NEW_MESSAGE :
                                              FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
                                          fdcan_buffer_index))
        ;
    // clang-format on

    return *this;
}

void *FDCAN::getSendFrame()
{
    return &sendFrame;
}

void *FDCAN::getReceiveFrame()
{
    return &receiveFrame;
}

uint8_t FDCAN::getState()
{
    return HAL_FDCAN_GetState(hcan);
}

uint8_t FDCAN::sendMessage()
{
    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &sendFrame.header,
                                         sendFrame.data);
}

uint8_t FDCAN::receiveMessage()
{
    return HAL_FDCAN_GetRxMessage(
        hcan,
        filter.FilterConfig == FDCAN_FILTER_TO_RXFIFO0 ? FDCAN_RX_FIFO0 :
                                                         FDCAN_RX_FIFO1,
        &receiveFrame.header, receiveFrame.data);
}

uint8_t FDCAN::sendReceiveMessage()
{
    return sendMessage() << 4 | receiveMessage();
}

FDCAN_FilterTypeDef &FDCAN::getFilter()
{
    return this->filter;
}

FDCAN &FDCAN::operator=(const FDCAN &other)
{
    hcan = other.hcan;
    fdcan_buffer_index = other.fdcan_buffer_index;
    filter = other.filter;

    return *this;
}

#endif
