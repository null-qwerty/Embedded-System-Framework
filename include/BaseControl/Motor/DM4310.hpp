#pragma once

#include "Motor.hpp"
#include "BaseControl/Connectivity/CAN/CAN.hpp"

class DM4310 : public Motor {
public:
#ifdef __CAN_H__
    DM4310(CAN &can, uint16_t send_id, uint16_t receive_id, int8_t cw);
#endif
    ~DM4310();

    DM4310 &init() final;
    DM4310 &deInit() final;

    DM4310 &encodeControlMessage() final;
    DM4310 &decodeFeedbackMessage() final;

private:
    uint8_t ifEnable = 0;

    const float DM4310_MAX_TAU = 10.f;
    const float DM4310_MAX_VEL = 30.f;

    uint16_t linearFloat2Uint(float x, float x_max, float x_min, uint8_t bits);
    float linearUint2Float(uint16_t x, float x_max, float x_min, uint8_t bits);

    float calculateControlData() final;

    DM4310 &encodeCAN(uint8_t *buffer);
    DM4310 &decodeCAN();
};
