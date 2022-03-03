#ifndef MPL3115A2_H
#define MPL3115A2_H
#include <Arduino.h>
#include <Wire.h> // for I2C communication
#include "I2Cdev.h"

#define MPL3115A2_ADDRESS 0x60 ///< default I2C address 1100000
#define MPL3115A2_WHOAMI 0x0C
#define MPL3115A2_CTRL_REG1 0x26
#define CTRL_REG_1 0x01
#define OFF_H 0x2D

class MPL3115A2
{

public:
    void init(int basis, unsigned long clockspeed, TwoWire *wire = &Wire);

    typedef struct
    {
        float raw;
        float smooth;
        float temp;
    } _baro;

    void setClock(uint32_t us);
    _baro read();

private:
    I2Cdev *i2c;
    uint32_t lastUpdateTime;
    uint32_t clockSpeed;
    _baro baro;
    static const byte bufferSize = 5;
    byte buffer[MPL3115A2::bufferSize];
    bool oneShot();
    float readBaro();
    float readAlt();
    float readTemp();
};

#endif