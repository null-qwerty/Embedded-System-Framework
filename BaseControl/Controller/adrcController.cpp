#include "BaseControl/Controller/adrcController.hpp"

adrcController::adrcController()
{
    init();
}

adrcController::adrcController(float wc, float b, float Umax, float Umin,
                               float r, float h)
    : wc(wc)
    , b(b)
    , Umax(Umax)
    , Umin(Umin)
    , r(r)
    , h(h)
{
    init();
}

adrcController::~adrcController()
{
}

adrcController &adrcController::init()
{
    wo = 10 * wc;
    beta = powf(M_E, -wo * h);
    kp = wc * wc;
    kd = 2 * wc;
    // clang-format off
    Phi <<                   3 * beta - 2                  , h, h * h / 2,
            -(1 - beta) * (1 - beta) * (5 + beta) / (2 * h), 1, h,
            -(1 - beta) * (1 - beta) * (1 - beta) / (h * h), 0, 1;

    H <<                 beta * beta * beta,                     0, 0,
            -3 * (1 - beta) * (1 - beta) * (1 + beta) / (2 * h), 1, 0,
                -(1 - beta) * (1 - beta) * (1 - beta) / (h * h), 0, 1;

    Tau << b * h * h / 2, 3 - 3 * beta,
            b * h       , (1 - beta) * (1 - beta) * (5 + beta) / (2 * h),
            0           , (1 - beta) * (1 - beta) * (1 - beta) / (h * h);

    J << 0, 1 - beta * beta * beta,
         0, 3 * (1 - beta) * (1 - beta) * (1 + beta) / (2 * h),
         0, (1 - beta) * (1 - beta) * (1 - beta) / (h * h);

    // clang-format on
    v << 0, 0;
    dv << 0, 0;
    z << 0, 0, 0;
    ud << 0, 0;
    yd << 0, 0, 0;

    return *this;
}

float adrcController::calculate(float ref, float fbk)
{
    dv[0] = v[1];
    dv[1] = -2 * r * v[1] - r * r * (v[0] - ref);
    v = v + dv * h;

    u0 = kp * (v(0) - yd(0)) + kd * (v(1) - yd(1));
    u = (u0 - yd(2)) / b;

    ud[0] = u;
    ud[1] = fbk; // u = [u, y]^T

    yd = Phi * z + J * ud;
    z = Phi * z + Tau * ud;

    if (u > Umax)
        u = Umax;
    else if (u < Umin)
        u = Umin;

    return u;
}