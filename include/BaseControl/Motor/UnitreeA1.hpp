#pragma once

#include "Motor.hpp"
#include "UnitreeA1protocol.hpp"

class UnitreeA1 : public Motor {
public:
    UnitreeA1(Connectivity &connectivity, uint16_t send_id, uint16_t receive_id,
              int8_t cw, float ratio = 9.1f, uint8_t option = MOTOR_OPTION_NONE,
              MotorOptionData optionData = {});
    ~UnitreeA1();

    UnitreeA1 &init() final;
    UnitreeA1 &deInit() final;

    UnitreeA1 &encodeControlMessage() final;
    UnitreeA1 &decodeFeedbackMessage() final;
    static uint32_t crc32_core(uint32_t *ptr, uint32_t len);

private:
    MotorState last_state;
    float calculateControlData() final;
};
