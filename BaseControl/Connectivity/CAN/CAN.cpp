#include "BaseControl/Connectivity/CAN/CAN.hpp"
#include "BaseControl/Connectivity/Connectivity.hpp"

#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_def.h"

CAN::CAN(CAN_HandleTypeDef *hcan, CAN_FilterTypeDef &filter)
    : Connectivity()
    , hcan(hcan)
    , filter(filter)
{
}

CAN::~CAN()
{
}

CAN &CAN::init()
{
    while (HAL_CAN_ConfigFilter(hcan, &filter) != HAL_OK)
        ;
    while (HAL_CAN_Start(hcan) != HAL_OK)
        ;
    while (HAL_CAN_ActivateNotification(
               hcan, filter.FilterFIFOAssignment == CAN_RX_FIFO0 ?
                         CAN_IT_RX_FIFO0_MSG_PENDING :
                         CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
        ;

    return *this;
}

void *CAN::getSendFrame()
{
    return &sendFrame;
}

void *CAN::getReceiveFrame()
{
    return &receiveFrame;
}

uint8_t CAN::getState()
{
    return HAL_CAN_GetState(hcan);
}

uint8_t CAN::sendMessage(void)
{
    return HAL_CAN_AddTxMessage(hcan, &sendFrame.header, sendFrame.data,
                                &can_tx_mailbox);
}

uint8_t CAN::receiveMessage(void)
{
    return HAL_CAN_GetRxMessage(hcan, filter.FilterFIFOAssignment,
                                &receiveFrame.header, receiveFrame.data);
}

uint8_t CAN::sendReceiveMessage(void)
{
    return sendMessage() << 4 | receiveMessage();
}

CAN_FilterTypeDef &CAN::getFilter()
{
    return this->filter;
}

CAN &CAN::operator=(const CAN &other)
{
    hcan = other.hcan;
    can_tx_mailbox = other.can_tx_mailbox;
    filter = other.filter;

    return *this;
}