#pragma once

class classicController {
public:
    virtual classicController &init() = 0;
    virtual float calculate(float ref, float fdb) = 0;
};