#ifndef I2C_h
#define I2C_h

#include <Arduino.h>
#include <Wire.h>

class I2C
{
public:
    I2C(uint8_t addr, uint8_t size, TwoWire *theWire = &Wire);
    byte read(byte regAddr);
    void write(byte regAddr, byte value);
    void readBytes();
    byte getBufferByte(uint8_t idx);

private:
    uint8_t address;
    TwoWire *wire;
    byte *buffer;
    uint8_t bufferSize;
};

#endif // I2C_h