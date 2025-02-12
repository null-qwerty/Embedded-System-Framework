#include "BaseControl/Controller/pidController.hpp"

#define abs(x) ((x) > 0 ? (x) : -(x))

pidController::pidController()
{
    up = 0.0;
    ui = 0.0;
    ud = 0.0;
    v1 = 0.0;
    i1 = 0.0;
    d1 = 0.0;
    d2 = 0.0;
    w1 = 1;
    Out = 0.0;
}

pidController::pidController(float Kp, float Ki, float Kd, float Umax,
                             float Umin, float Kr, float Km, float Kiae)
    : Kr(Kr)
    , Kp(Kp)
    , Ki(Ki)
    , Kd(Kd)
    , Km(Km)
    , Umax(Umax)
    , Umin(Umin)
    , Kiae(Kiae)
{
    up = 0.0;
    ui = 0.0;
    ud = 0.0;
    v1 = 0.0;
    i1 = 0.0;
    d1 = 0.0;
    d2 = 0.0;
    w1 = 1;
    Out = 0.0;
}

pidController &pidController::init()
{
    up = 0.0;
    ui = 0.0;
    ud = 0.0;
    v1 = 0.0;
    i1 = 0.0;
    d1 = 0.0;
    d2 = 0.0;
    w1 = 1;
    Out = 0.0;

    return *this;
}
float pidController::Calculate(float Ref, float Fbk)
{
    // clang-format off
    // v.data.up = _IQmpy(v.param.Kr, v.term.Ref) - v.term.Fbk;
    up = Kr * Ref - Fbk;
    // v.data.ui = _IQmpy(v.param.Ki, _IQmpy(v.data.w1, (v.term.Ref - v.term.Fbk))) + v.data.i1
    // v.data.i1 = v.data.ui
    ui = Ki * (w1 * (Ref - Fbk)) + i1;
    i1 = ui;
    // v.data.d2 = _IQmpy(v.param.Kd, _IQmpy(v.term.c1, (_IQmpy(v.term.Ref, v.param.Km) - v.term.Fbk))) - v.data.d2
    // v.data.ud = v.data.d2 + v.data.d1
    // v.data.d1 = _IQmpy(v.data.ud, v.term.c2)
    // d2 = Kd * (c1 * (Km * Ref - Fbk)) - d2;
    // ud = d2 + d1;
    // 参考 https://e2e.ti.com/support/microcontrollers/c2000-microcontrollers-group/c2000/f/c2000-microcontrollers-forum/1034536/tms320f28388d-a-question-of-runpid-function-in-the-pid_grando-h
    // 修改如下
    d0 = Kd * c1 * (Km * Ref - Fbk);
    d2 = d0 - d01;
    d01 = d0;
    ud = d2 + d1;
    d1 = c2 * ud;

    // v.data.v1 = _IQmpy(v.param.Kp, (v.data.up + v.data.ui + v.data.ud))
    v1 = Kp * (up + ui + ud);
    // v.term.Out= _IQsat(v.data.v1, v.param.Umax, v.param.Umin)
    if (v1 > Umax)
        Out = Umax;
    else if (v1 < Umin)
        Out = Umin;
    else
        Out = v1;
    // v.data.w1 = (v.term.Out == v.data.v1) ? _IQ(1.0) : _IQ(0.0)
    w1 = ((Out == v1) ? 1 : 0);
    // clang-format on

    Iae = Iae + Kiae * abs(Ref - Fbk);

    return Out;
}

pidController &pidController::setFilter(float a, float T)
{
    c1 = a;
    c2 = 1. - c1 * T;
    return *this;
}

pidController &pidController::setPidParam(float Kp, float Ki, float Kd,
                                          float Kr, float Km)
{
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->Kr = Kr;
    this->Km = Km;
    return *this;
}

pidController &pidController::setOutLimit(float Umax, float Umin)
{
    this->Umax = Umax;
    this->Umin = Umin;
    return *this;
}

pidController &pidController::setIaeParam(float Kiae)
{
    this->Kiae = Kiae;
    return *this;
}