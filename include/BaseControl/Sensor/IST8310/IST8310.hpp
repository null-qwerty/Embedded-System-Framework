#pragma once

#include "BaseControl/Sensor/Sensor.hpp"

class IST8310 : public Sensor {
public:
    typedef struct Data_s {
        float x;
        float y;
        float z;
        float temprature;
    } Data_t;
    IST8310(Connectivity &connectivity, uint8_t address);
    ~IST8310() = default;

    virtual IST8310 &init() override;
    virtual void *getData() override;
    virtual IST8310 &reset() override;

private:
    Data_t data;
    uint8_t address;
    uint8_t dataBuffer[8];

    uint8_t writeByte(uint8_t reg, uint8_t data);
    uint8_t readByte(uint8_t reg, uint8_t *data);
    uint8_t readData(uint8_t reg, uint8_t *data, uint8_t lenth);
};