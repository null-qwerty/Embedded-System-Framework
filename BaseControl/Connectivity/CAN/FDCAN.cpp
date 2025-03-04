#include "BaseControl/Connectivity/CAN/FDCAN.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"

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
    while (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
        ;
    while (HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT,
                                        FDCAN_REJECT_REMOTE,
                                        FDCAN_REJECT_REMOTE))
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
    return HAL_FDCAN_AddMessageToTxBuffer(hcan, &sendFrame.header,
                                          sendFrame.data, fdcan_buffer_index);
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
