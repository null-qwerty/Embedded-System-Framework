#include "Utils/note.hpp"

#include "Utils/buzzer.hpp"

Note::Note()
{
}

Note::Note(int frequency, int duration)
    : frequency(frequency)
    , duration(duration)
{
}

Note::~Note()
{
}

Note &Note::play()
{
    buzzer_on(frequency, MIN_BUZZER_PWM);
    HAL_Delay(duration);
    buzzer_off();
    return *this;
}