#include "BaseControl/Connectivity/UART/DBUS.hpp"

DBUS::DBUS(UART_HandleTypeDef *huart)
    : UART(huart)
{
    ((xUARTFrame_t *)this->getReceiveFrame())->data = dbusBuffer;
    ((xUARTFrame_t *)this->getReceiveFrame())->length = DBUS_BUFFER_SIZE;
}

void DBUS::decodeDBUSMessage()
{
    // clang-format off
    dbusData.rc.ch0 = (((int16_t)dbusBuffer[0] >> 0) | ((int16_t)dbusBuffer[1] << 8)) & 0x07FF;
    dbusData.rc.ch1 = (((int16_t)dbusBuffer[1] >> 3) | ((int16_t)dbusBuffer[2] << 5)) & 0x07FF;
    dbusData.rc.ch2 = (((int16_t)dbusBuffer[2] >> 6) | ((int16_t)dbusBuffer[3] << 2) | ((int16_t)dbusBuffer[4] << 10)) & 0x07FF;
    dbusData.rc.ch3 = (((int16_t)dbusBuffer[4] >> 1) | ((int16_t)dbusBuffer[5] << 7)) & 0x07FF;

    dbusData.rc.s1 = dbusBuffer[5] >> 6;
    dbusData.rc.s2 = ((dbusBuffer[5] >> 4) & 0x0003);

    dbusData.mouse.x = ((int16_t)dbusBuffer[6])  | ((int16_t)dbusBuffer[7] << 8);
    dbusData.mouse.y = ((int16_t)dbusBuffer[8])  | ((int16_t)dbusBuffer[9] << 8);
    dbusData.mouse.z = ((int16_t)dbusBuffer[10]) | ((int16_t)dbusBuffer[11] << 8);
    dbusData.mouse.press_l = dbusBuffer[12];
    dbusData.mouse.press_r = dbusBuffer[13];

    dbusData.key.v = ((int16_t)dbusBuffer[14]) | ((int16_t)dbusBuffer[15] << 8);

    dbusData.rc.tw = (((int16_t)dbusBuffer[16] >> 0) | ((int16_t)dbusBuffer[17] << 8)) & 0x07FF;
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