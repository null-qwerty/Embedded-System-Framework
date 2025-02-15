#pragma once

#include "main.h"

#define MAX_BUZZER_PWM 999
#define MIN_BUZZER_PWM 700

#define proport 84000 // Tclk/(arr+1)=84000000/(1000)
#define FREQUENCY_TO_ARR(f) ((proport / f) - 1)

/**
 * @brief 打开蜂鸣器
 *
 * @param psc 用于设置预分频器的值，调节频率
 * @param pwm 用于设置占空比，调节音量
 */
void buzzer_on(uint16_t psc, uint16_t pwm);
/**
 * @brief 关闭蜂鸣器
 *
 */
void buzzer_off(void);
