#include "BaseControl/Sensor/BMI088/BMI088.hpp"
#include "BaseControl/Sensor/BMI088/BMI088_reg.hpp"

#include "BaseControl/Connectivity/SPI/SPI.hpp"

#include "Utils/Utils.hpp"

BMI088::BMI088(Connectivity &connectivity)
    : Sensor(connectivity)
{
}

Sensor &BMI088::init()
{
    initAcc();
    initGyro();
    return *this;
}

void *BMI088::getData()
{
    readAccel();
    readGyro();
    readTemprature();

    return &data;
}

BMI088 &BMI088::reset()
{
    // 加速度计复位，寄存器重置，重置完后加速度计默认为休眠状态
    writeDataToAcc(BMI088_ACC_SOFTRESET_REG, BMI088_ACC_SOFTRESET_VALUE);
    delay_us(1000);
    // 陀螺仪复位，寄存器重置，重置完后陀螺仪默认为开启状态
    writeDataToGyro(BMI088_GYRO_SOFTRESET_REG, BMI088_GYRO_SOFTRESET_VALUE);
    delay_us(50000);
    init();
    return *this;
}

BMI088 &BMI088::initAcc()
{
    // 由于 SPI 通信时，发送同时并不能接收到需要的数据，所以进行两次读取
    readDataFromAcc(BMI088_CHIP_ID_REG, &accChipID);
    readDataFromAcc(BMI088_CHIP_ID_REG, &accChipID);
    // 加速度计默认为休眠状态，需要设置为激活状态
    writeDataToAcc(BMI088_ACC_PWR_CFG_REG, BMI088_ACC_PWR_CFG_ACTIVE);
    delay_us(1000);
    // 设置加速度计量程为 ±3g，带宽为正常模式，输出数据速率为 1600Hz
    writeDataToAcc(BMI088_ACC_RANGE_REG, BMI088_ACC_RANGE_3G);
    delay_us(1000);
    writeDataToAcc(BMI008_ACC_CONF_REG, (1 << 7) |
                                            (BMI088_ACC_CONF_BWP_NORM << 4) |
                                            (BMI088_ACC_CONF_ODR_1600_Hz));
    // 设置加速度计为激活状态
    writeDataToAcc(BMI088_ACC_PWR_CTRL_REG, BMI088_ACC_PWR_CTRL_ON);
    delay_us(1000);
    // 读取加速度计芯片 ID，正常情况下应该为 0x1E
    readDataFromAcc(BMI088_CHIP_ID_REG, &accChipID);
    readDataFromAcc(BMI088_CHIP_ID_REG, &accChipID);

    return *this;
}

BMI088 &BMI088::initGyro()
{
    readDataFromGyro(BMI088_CHIP_ID_REG, &gyroChipID);
    readDataFromGyro(BMI088_CHIP_ID_REG, &gyroChipID);
    // 设置陀螺仪量程为 ±2000°/s
    writeDataToGyro(BMI088_GYRO_RANGE_REG, BMI088_GYRO_RANGE_2000_DEG_S);
    delay_us(1000);
    // 设置陀螺仪输出数据速率为 1000Hz，滤波器带宽为 116Hz
    writeDataToGyro(BMI088_GYRO_BANDWIDTH_REG,
                    BMI088_GYRO_ODR_1000Hz_BANDWIDTH_116Hz);
    delay_us(1000);
    // 读取陀螺仪芯片 ID，正常情况下应该为 0x0F
    readDataFromGyro(BMI088_CHIP_ID_REG, &gyroChipID);
    readDataFromGyro(BMI088_CHIP_ID_REG, &gyroChipID);

    return *this;
}
uint8_t BMI088::readAccel()
{
    uint8_t ret = 0;
    /*
     * 和读取角速度一样，加速度数据也可一次性读取，但第一个字节是无效的。
     * 参考 BMI088 数据手册 6.1.2 节：
     * 6.1.2 SPI interface of accelerometer part
     *
     * In case of read operations of the acclerometer part, the requested data
     * is not sent immediately, but instead first a dummy byte is sent, and
     * after this dummy byte the actual request register content is transmitted.
     *
     * This means that - in contrast to the description in section 6.1.1 - a
     * single byte read operation requires to read 2 bytes in burst mode, of
     * which the first received byte can be discared, while the second byte
     * contains the desired data.
     *
     * the same applies to burst-read operations. For example, to read the
     * accelerometer values in SPI mode, the user has to read 7 bytes, starting
     * from address 0x12 (ACC data). From these bytes the user must discard the
     * first byte and finds the acceleration information in byte #2 - #7
     * (corresponding to the content of the addresses 0x12 - 0x17).
     *
     * The data bits are used as follows:
     * - Bit #0: Read/Write bit. When 0, the data SDI is written into the chip.
     *           When 1, the data SDO from the chip is read.
     * - Bit #1-7: Address AD(6:0).
     * - Bit #8-15:
     *     - When in write mode, these are the data SDI, which will be written
     *       into the address.
     *     - When in read mode, these bits contain unperdictable values, and the
     *       user has to read Bit #16-23 to get the actual data from the reading
     *       address.
     */
    ENABLE_ACC();
    uint8_t add = BMI088_ACC_OUT_REG | BMI088_READ;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->data = &add;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->length = 1;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->data = accBuffer;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->length = 7;
    connectivity.sendMessage();
    ret = connectivity.receiveMessage();
    DISABLE_ACC();

    data.accel.x =
        (int16_t)((accBuffer[2] << 8) | accBuffer[1]) * BMI088_ACC_SEN;
    data.accel.y =
        (int16_t)((accBuffer[4] << 8) | accBuffer[3]) * BMI088_ACC_SEN;
    data.accel.z =
        (int16_t)((accBuffer[6] << 8) | accBuffer[5]) * BMI088_ACC_SEN;

    return ret;
}

uint8_t BMI088::readGyro()
{
    uint8_t ret = 0;
    /*
     * 保持 CSB 为低电平，可以一次性读取所有的角速度数据，参考 BMI088
     * 数据手册 6.1.1 节：
     * 6.1.1 SPI interface of gyroscope part
     *
     * For single byte read as well as write operations, 16-bit protocols are
     * used. The SPI interface also supports multiple-byte read
     * operations(burst-read).
     *
     * The communication starts when the CSB (1 or 2) is pulled low by the SPI
     * master and stops when CSB (1 or 2) is pulled high. SCK is also controlled
     * by SPI master. SDI and SDO (1 or 2) are driven at the falling edge of SCK
     * and should be captured at the rising edge of SCK.
     *
     * the data bits are used as follows:
     * - Bit #0: Read/Write bit. When 0, the data SDI is weitten into
     *           the chip. When 1, the data SDO from the chip is read.
     * - Bit #1-7: Address AD(6:0).
     * - Bit #8-15: when in write mode, these are the data SDI, which will be
     *              written into the address. When in read mode, these are the
     *              data SDO, which are read from the address.
     *
     * Multiple read operations (burst-read) are possible by keeping CSB low and
     * continuing the data transfer (i.e. continuing to toggle SCK). Only the
     * first register address has to be written. Addresses are automatically
     * incremented after each read access as lang as CSB stays active low.
     */
    ENABLE_GYRO();
    uint8_t add = BMI088_GYRO_OUT_REG | BMI088_READ;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->data = &add;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->length = 1;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->data = gyroBuffer;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->length = 6;
    connectivity.sendMessage();
    ret = connectivity.receiveMessage();
    DISABLE_GYRO();

    data.gyro.roll =
        (int16_t)((gyroBuffer[1] << 8) | gyroBuffer[0]) * BMI088_GYRO_SEN;
    data.gyro.pitch =
        (int16_t)((gyroBuffer[3] << 8) | gyroBuffer[2]) * BMI088_GYRO_SEN;
    data.gyro.yaw =
        (int16_t)((gyroBuffer[5] << 8) | gyroBuffer[4]) * BMI088_GYRO_SEN;

    return ret;
}

uint8_t BMI088::readTemprature()
{
    uint8_t ret = 0;

    ENABLE_ACC();
    uint8_t add = BMI088_TEMPRATURE_OUT_REG | BMI088_READ;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->data = &add;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->length = 1;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->data =
        tempratureBuffer;
    // 同加速度计，第一个字节是无效的
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->length = 3;
    connectivity.sendMessage();
    ret = connectivity.receiveMessage();
    DISABLE_ACC();

    int16_t temp =
        ((int16_t)(tempratureBuffer[1] << 3) | (tempratureBuffer[2] >> 5));
    if (temp > 1023)
        temp -= 2048;
    data.temprature = temp * 0.125f + 23.0f;

    return ret;
}

uint8_t BMI088::readByte(uint8_t reg, uint8_t *data)
{
    reg |= BMI088_READ;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->data = &reg;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->length = 1;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->data = data;
    ((SPI::xSPIFrame_t *)connectivity.getReceiveFrame())->length = 1;
    connectivity.sendMessage();
    return connectivity.receiveMessage();
}

uint8_t BMI088::writeByte(uint8_t reg, uint8_t data)
{
    reg |= BMI088_WRITE;
    uint8_t sendBuffer[] = { reg, data };
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->data = sendBuffer;
    ((SPI::xSPIFrame_t *)connectivity.getSendFrame())->length = 2;
    return connectivity.sendMessage();
}

uint8_t BMI088::writeDataToAcc(uint8_t reg, uint8_t data)
{
    ENABLE_ACC();
    uint8_t ret = writeByte(reg, data);
    DISABLE_ACC();
    return ret;
}

uint8_t BMI088::writeDataToGyro(uint8_t reg, uint8_t data)
{
    ENABLE_GYRO();
    uint8_t ret = writeByte(reg, data);
    DISABLE_GYRO();
    return ret;
}

uint8_t BMI088::readDataFromAcc(uint8_t reg, uint8_t *data)
{
    ENABLE_ACC();
    uint8_t ret = readByte(reg, data);
    DISABLE_ACC();
    return ret;
}

uint8_t BMI088::readDataFromGyro(uint8_t reg, uint8_t *data)
{
    ENABLE_GYRO();
    uint8_t ret = readByte(reg, data);
    DISABLE_GYRO();
    return ret;
}