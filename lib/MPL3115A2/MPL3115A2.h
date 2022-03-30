#ifndef MPL3115A2_H
#define MPL3115A2_H
#include <Arduino.h>
#include <Wire.h> // for I2C communication
#include "IBaro.h"

#define MPL3115A2_ADDRESS 0x60 ///< default I2C address 1100000
#define MPL3115A2_WHOAMI 0x0C
#define MPL3115A2_CTRL_REG1 0x26
#define CTRL_REG_1 0x01
#define OFF_H 0x2D

class MPL3115A2 : public IBaro
{

public:
    MPL3115A2();
    void init(int basis, unsigned long clockspeed, TwoWire *wire = &Wire);
    float read();

private:
    uint8_t buffer[5] = {0, 0, 0, 0, 0};
    float readBaro();
    float readAlt();
    float readTemp();
};

#endif