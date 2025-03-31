#include "Math/Trigonometric.hpp"
#include "arm_math.h"
#include "arm_math_types.h"

namespace Math
{
float32_t sin(float32_t x)
{
    return arm_sin_f32(x);
}

float32_t cos(float32_t x)
{
    return arm_cos_f32(x);
}

float32_t atan2(float32_t y, float32_t x)
{
    float32_t res;
    arm_atan2_f32(y, x, &res);
    return res;
}
}