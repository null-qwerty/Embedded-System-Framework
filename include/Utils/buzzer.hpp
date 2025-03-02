#pragma once

#include "main.h"

#include "note.hpp"

#ifdef __TIM_H__

class Buzzer {
public:
    Buzzer(TIM_HandleTypeDef *htim, uint16_t channel, uint32_t clock_speed,
           uint16_t pwm_max);
    Buzzer &on(uint16_t freq, float loudness);
    Buzzer &play(Note &note);
    Buzzer &off();

private:
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t proport;
    uint32_t maxPwm;
};
#endif