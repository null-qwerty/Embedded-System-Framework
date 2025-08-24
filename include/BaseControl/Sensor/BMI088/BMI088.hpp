#pragma once

#include "BaseControl/Sensor/Sensor.hpp"

class BMI088 : public Sensor {
public:
    typedef struct Data_s {
        struct accel_t {
            float x;
            float y;
            float z;
        } accel;
        struct gyro_t {
            float roll;
            float pitch;
            float yaw;
        } gyro;
        float temperature;
    } Data_t;

    BMI088(Connectivity &connectivity);
    ~BMI088() = default;

    virtual Sensor &init() override;
    virtual void *getData() override;
    virtual BMI088 &reset() override;

    BMI088 &setGyroTemperatureCompensation(float kyaw, float kpitch,
                                           float kroll, float byaw,
                                           float bpitch, float broll);

private:
    Data_t data = {};
    uint8_t accBuffer[7] = {}, gyroBuffer[6] = {}, temperatureBuffer[3] = {};
    uint8_t accChipID = 0, gyroChipID = 0;
    float kyaw = 0.0, kpitch = 0.0, kroll = 0.0;
    float byaw = 0.0, bpitch = 0.0, broll = 0.0;

    BMI088 &initAcc();
    BMI088 &initGyro();

    uint8_t readAccel();
    uint8_t readGyro();
    uint8_t readTemperature();

    uint8_t readByte(uint8_t reg, uint8_t *data);
    uint8_t writeByte(uint8_t reg, uint8_t data);

    uint8_t writeDataToAcc(uint8_t reg, uint8_t data);
    uint8_t writeDataToGyro(uint8_t reg, uint8_t data);
    uint8_t readDataFromAcc(uint8_t reg, uint8_t *data);
    uint8_t readDataFromGyro(uint8_t reg, uint8_t *data);
};