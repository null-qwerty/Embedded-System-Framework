#pragma once

#include "BaseControl/Connectivity/Connectivity.hpp"

class Sensor {
public:
    Sensor(Connectivity &connectivity);
    ~Sensor() = default;

    virtual Sensor &init() = 0;
    virtual void *getData() = 0;
    virtual Sensor &reset() = 0;

protected:
    Connectivity &connectivity;
};