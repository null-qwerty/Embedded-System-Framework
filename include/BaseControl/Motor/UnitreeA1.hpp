#pragma once

#include "Motor.hpp"
#include "UnitreeA1protocol.hpp"

class UnitreeA1 : public Motor {
public:
    UnitreeA1(Connectivity &connectivity, uint16_t send_id, uint16_t receive_id,
              int8_t cw, float ratio = 9.1f);
    ~UnitreeA1();

    UnitreeA1 &init() final;
    UnitreeA1 &deInit() final;

    UnitreeA1 &encodeControlMessage() final;
    UnitreeA1 &decodeFeedbackMessage() final;

private:
    uint8_t ifEnable = 0;

    float calculateControlData() final;

    uint32_t crc32_core(uint32_t *ptr, uint32_t len);
};
