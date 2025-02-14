#include "BaseControl/Connectivity/UART/DBUS.hpp"

DBUS::DBUS(UART_HandleTypeDef *huart, dmaOption dma)
    : UART(huart, dma)
{
    ((xUARTFrame_t *)this->getReceiveFrame())->data[0] = dbusBuffer[0];
    ((xUARTFrame_t *)this->getReceiveFrame())->data[1] = dbusBuffer[1];
    ((xUARTFrame_t *)this->getReceiveFrame())->length = DBUS_BUFFER_SIZE;
}

void DBUS::decodeDBUSMessage()
{
    auto readIndex = ((xUARTFrame_t *)this->getReceiveFrame())->readIndex;
    // clang-format off
    dbusData.rc.ch0 = (((int16_t)dbusBuffer[readIndex][0] >> 0) | ((int16_t)dbusBuffer[readIndex][1] << 8)) & 0x07FF;
    dbusData.rc.ch1 = (((int16_t)dbusBuffer[readIndex][1] >> 3) | ((int16_t)dbusBuffer[readIndex][2] << 5)) & 0x07FF;
    dbusData.rc.ch2 = (((int16_t)dbusBuffer[readIndex][2] >> 6) | ((int16_t)dbusBuffer[readIndex][3] << 2) | ((int16_t)dbusBuffer[readIndex][4] << 10)) & 0x07FF;
    dbusData.rc.ch3 = (((int16_t)dbusBuffer[readIndex][4] >> 1) | ((int16_t)dbusBuffer[readIndex][5] << 7)) & 0x07FF;

    dbusData.rc.s1 = dbusBuffer[readIndex][5] >> 6;
    dbusData.rc.s2 = ((dbusBuffer[readIndex][5] >> 4) & 0x0003);

    dbusData.mouse.x = ((int16_t)dbusBuffer[readIndex][6])  | ((int16_t)dbusBuffer[readIndex][7] << 8);
    dbusData.mouse.y = ((int16_t)dbusBuffer[readIndex][8])  | ((int16_t)dbusBuffer[readIndex][9] << 8);
    dbusData.mouse.z = ((int16_t)dbusBuffer[readIndex][10]) | ((int16_t)dbusBuffer[readIndex][11] << 8);
    dbusData.mouse.press_l = dbusBuffer[readIndex][12];
    dbusData.mouse.press_r = dbusBuffer[readIndex][13];

    dbusData.key.v = ((int16_t)dbusBuffer[readIndex][14]) | ((int16_t)dbusBuffer[readIndex][15] << 8);

    dbusData.rc.tw = (((int16_t)dbusBuffer[readIndex][16] >> 0) | ((int16_t)dbusBuffer[readIndex][17] << 8)) & 0x07FF;
    // clang-format on
    ifDataValid();
}

DBUS::DBUSData &DBUS::getDBUSData()
{
    return dbusData;
}

bool DBUS::ifDataValid()
{
    bool status = true;

    if (dbusData.rc.ch0 > RC_CH_VALUE_MAX || dbusData.rc.ch0 < RC_CH_VALUE_MIN)
        (dbusData.rc.ch0 = RC_CH_VALUE_OFFSET, status = false);
    if (dbusData.rc.ch1 > RC_CH_VALUE_MAX || dbusData.rc.ch1 < RC_CH_VALUE_MIN)
        (dbusData.rc.ch1 = RC_CH_VALUE_OFFSET, status = false);
    if (dbusData.rc.ch2 > RC_CH_VALUE_MAX || dbusData.rc.ch2 < RC_CH_VALUE_MIN)
        (dbusData.rc.ch2 = RC_CH_VALUE_OFFSET, status = false);
    if (dbusData.rc.ch3 > RC_CH_VALUE_MAX || dbusData.rc.ch3 < RC_CH_VALUE_MIN)
        (dbusData.rc.ch3 = RC_CH_VALUE_OFFSET, status = false);
    if (dbusData.rc.s1 > RC_SW_MID)
        (dbusData.rc.s1 = 0, status = false);
    if (dbusData.rc.s1 < RC_SW_UP)
        (dbusData.rc.s1 = 0, status = false);
    if (dbusData.rc.s2 > RC_SW_MID)
        (dbusData.rc.s2 = 0, status = false);
    if (dbusData.rc.s2 < RC_SW_UP)
        (dbusData.rc.s2 = 0, status = false);
    if (dbusData.rc.tw > RC_TW_VALUE_MAX || dbusData.rc.tw < RC_TW_VALUE_MIN)
        (dbusData.rc.tw = RC_TW_VALUE_OFFSET, status = false);

    return status;
}