#pragma once

#include <BaseControl/Connectivity/UART/UART.hpp>

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* ----------------------- RC Thumbwheel Definition------------------------- */
#define RC_TW_VALUE_MIN ((uint16_t)364)
#define RC_TW_VALUE_OFFSET ((uint16_t)1024)
#define RC_TW_VALUE_MAX ((uint16_t)1556)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)0x01 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)0x01 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)0x01 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)0x01 << 3)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)0x01 << 4)
#define KEY_PRESSED_OFFSET_E ((uint16_t)0x01 << 5)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)0x01 << 6)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)0x01 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)0x01 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)0x01 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)0x01 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)0x01 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)0x01 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)0x01 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)0x01 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)0x01 << 15)

#define DBUS_BUFFER_SIZE 18

class DBUS : public UART {
public:
    typedef struct DBUSData_s {
        struct {
            uint16_t ch0 = 1024;
            uint16_t ch1 = 1024;
            uint16_t ch2 = 1024;
            uint16_t ch3 = 1024;
            uint8_t s1 = 0;
            uint8_t s2 = 0;
            uint16_t tw = 1024;
        } rc;
        struct {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
            uint8_t press_l = 0;
            uint8_t press_r = 0;
        } mouse;
        struct {
            uint16_t v = 0;
        } key;
    } DBUSData;

    DBUS(UART_HandleTypeDef *huart, dmaOption dma = DISABLE);

    void decodeDBUSMessage();

    DBUSData &getDBUSData();

private:
    DBUSData dbusData = {};
    uint8_t dbusBuffer[2][DBUS_BUFFER_SIZE] = {};

    bool ifDataValid();
};