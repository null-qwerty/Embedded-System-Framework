#include "Utils/buzzer.hpp"

#ifdef __TIM_H__

Buzzer::Buzzer(TIM_HandleTypeDef *htim, uint16_t channel, uint32_t clock_speed,
               uint16_t pwm_max)
    : htim(htim)
    , channel(channel)
    , proport(clock_speed / 1000)
    , maxPwm(pwm_max)
{
}

Buzzer &Buzzer::on(uint16_t freq, float loudness)
{
    if (freq == 0) {
        this->off();
        return *this;
    }
    if (loudness >= 1) {
        loudness = 1;
    } else if (loudness <= 0) {
        loudness = 0;
    }

    uint16_t pwm = loudness * maxPwm;

    __HAL_TIM_PRESCALER(htim, (proport / freq) - 1);
    __HAL_TIM_SetCompare(htim, channel, pwm);

    return *this;
}

Buzzer &Buzzer::play(Note &note)
{
    this->on(note.frequency, note.loudness);
    HAL_Delay(note.duration);
    this->off();
    HAL_Delay(1);

    return *this;
}

Buzzer &Buzzer::off()
{
    __HAL_TIM_SetCompare(htim, channel, 0);

    return *this;
}
#endif