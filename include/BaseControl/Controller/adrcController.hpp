#pragma once

#include "classicController.hpp"
#include "Math/Matrix.hpp"
#include "Math/Vector.hpp"

class adrcController : public classicController {
public:
    adrcController();
    adrcController(float wc, float b, float Umax, float Umin, float r = 1000.,
                   float h = 0.001);
    ~adrcController();

    adrcController &init() override;

    float calculate(float ref, float fbk) override;

private:
    float wc, wo, b, r, h;
    float Umax, Umin;
    float u0, u;

    float beta;

    float kp, kd;

    Vector2f v;
    Vector2f dv;

    Vector3f z;
    Vector2f ud;
    Vector3f yd;

    Matrix3x3f Phi, H;
    Matrix<3, 2> Tau, J;
};