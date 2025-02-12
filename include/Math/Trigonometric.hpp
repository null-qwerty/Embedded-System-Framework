#pragma once

#include "arm_math.h"

#define DEGREE_TO_RAND(degree) ((float)(degree) * PI / 180.f)
#define RAND_TO_DEGREE(rand) ((float)(rand) * 180.f / PI)

float32_t sin(float32_t x);
float32_t cos(float32_t x);
float32_t atan2(float32_t y, float32_t x);