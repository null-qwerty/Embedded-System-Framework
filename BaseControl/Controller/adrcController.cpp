#include "BaseControl/Controller/adrcController.hpp"

#include "Math/Math.hpp"

adrcController::adrcController(float h, float r, float b, float delta,
                               float beta01, float beta02, float beta03,
                               float alpha1, float alpha2, float beta1,
                               float beta2)
    : h(h)
    , r(r)
    , x1(0)
    , x2(0)
    , b(b)
    , delta(delta)
    , beta01(beta01)
    , beta02(beta02)
    , beta03(beta03)
    , z1(0)
    , z2(0)
    , z3(0)
    , alpha1(alpha1)
    , alpha2(alpha2)
    , beta1(beta1)
    , beta2(beta2)
    , u(0)
{
}

float adrcController::calculate(float ref, float fbk)
{
    // TD
    float fh;
    fh = fhan(x1 - ref, x2, r, h);
    x1 += h * x2;
    x2 += h * fh;
    // ESO
    // float e, fe, fe1;
    e = z1 - fbk;
    fe = fal(e, 0.5, delta);
    fe1 = fal(e, 0.25, delta);
    z1 += h * (z2 - beta01 * e);
    z2 += h * (z3 - beta02 * fe + b * u);
    z3 += h * (-beta03 * fe1);
    // NLSEF
    float e1, e2, u0;
    e1 = x1 - z1;
    e2 = x2 - z2;
    u0 = beta1 * fal(e1, alpha1, delta) + beta2 * fal(e2, alpha2, delta);

    // 控制输出
    u = u0 - z3 / b;

    return u;
}

int adrcController::sign(float x)
{
    return (x > 0) - (x < 0);
}

float adrcController::fsg(float x, float y)
{
    return (sign(x + y) - sign(x - y)) / 2.0;
}

float adrcController::fhan(float x1, float x2, float r, float h)
{
    // float d, a0, y, a1, a2, sy, a, sa;

    // d = r * h * h;
    // a0 = h * x2;
    // y = x1 + a0;
    // a1 = sqrt(d * (d + 8 * fabs(y)));
    // a2 = a0 + sign(y) * (a1 - d) / 2;
    // sy = fsg(y, d);
    // a = (a0 + y - a2) * sy + a2;
    // sa = fsg(a, d);

    // return -r * (a / d - sign(a)) * sa - r * sign(a);
    float d, d0, y, a0, a;

    d = r * h;
    d0 = d * h;
    y = x1 + h * x2;
    a0 = sqrtf(d * d + 8 * r * fabs(y));
    a = fabs(y) > d0 ? x2 + (a0 - d) / 2.0 * sign(y) : x2 + y / h;

    return fabs(a) <= d ? -r * a / d : -r * sign(a);
}

float adrcController::fal(float x, float a, float delta)
{
    if (fabs(x) < delta) {
        return x / pow(delta, 1 - a);
    } else {
        return pow(fabs(x), a) * sign(x);
    }
}