#ifndef BMP390_H
#define BMP390_H
#include <Arduino.h>
#include <Wire.h> // for I2C communication
#include "I2Cdev.h"

#define BMP390_ADDRESS 0x77 ///< default I2C address
#define BMP390_CMD_ADDRESS 0x7E
#define BMP390_CTRL_REG1 0x26
#define CTRL_REG_1 0x01
#define OFF_H 0x2D
#define BMP390_CHIP_ID_ADDRESS 0x00
#define BMP390_CHIP_ID 0x60
#define BMP390_RESET_CMD 0xB6
#define BMP390_STATUS_ADDRESS 0x03
#define BMP390_CMD_READY 0x10
#define BMP390_DATA_REG_0 0x04
#define BMP3_CALIBRATION_DATA_ADDRESS 0x31

class BMP390
{

public:
    BMP390();
    void init(int basis, TwoWire *wire = &Wire);
    float read();

private:
    I2Cdev *i2c;
    float readBaro();
    float readRawBaro();
    void calibrate(const uint8_t *reg_data);
    float seaLevelPressure;
    float alt;
    static int8_t calCrc(uint8_t seed, uint8_t data);
    static int8_t validateTrim(struct bmp3_dev *dev);

    struct QuantizedCalibrationData
    {
        double t1;
        double t2;
        double t3;
        double p1;
        double p2;
        double p3;
        double p4;
        double p5;
        double p6;
        double p7;
        double p8;
        double p9;
        double p10;
        double p11;
        double lin;
    } quantizedCalibrationData;

    struct RegisterCalibrationData
    {
        uint16_t t1;
        uint16_t t2;
        int8_t t3;
        int16_t p1;
        int16_t p2;
        int8_t p3;
        int8_t p4;
        uint16_t p5;
        uint16_t p6;
        int8_t p7;
        int8_t p8;
        int16_t p9;
        int8_t p10;
        int8_t p11;
        int64_t lin;
    } registerCalibrationData;
};

#endif