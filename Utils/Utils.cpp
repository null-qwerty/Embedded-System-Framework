#include "Utils/Utils.hpp"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

void delay_us(uint32_t us)
{
    if (us > 65535)
        us = 65535;
    HAL_TIM_Base_Start(&htim7);
    __HAL_TIM_SetCounter(&htim7, 0);
    while (__HAL_TIM_GetCounter(&htim7) < us)
        ;
    HAL_TIM_Base_Stop(&htim7);
}
