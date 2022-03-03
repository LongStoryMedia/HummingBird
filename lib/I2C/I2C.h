#ifndef I2C_h
#define I2C_h

#include <Arduino.h>
#include <Wire.h>

class I2C
{
public:
    I2C(byte addr, byte size, TwoWire *theWire = &Wire);
    byte read(byte regAddr);
    bool write(byte regAddr, byte *value, byte length = 0);
    bool write(byte regAddr, byte value);
    void readBytes();
    byte getBufferByte(byte idx);
    byte readBits(byte regAddr, byte bitStart, byte length, unsigned short timeout = 0);
    byte readByte(byte regAddr, unsigned short timeout = 0);
    byte readSomeBytes(byte regAddr, byte length, unsigned short timeout = 0);
    byte readBit(byte regAddr, byte bitNum, unsigned long timeout = 0);
    bool writeBits(byte regAddr, byte bitStart, byte length, byte data);
    bool writeBit(byte regAddr, byte bitNum, byte data);
    void clearBuffer();

private:
    byte address;
    TwoWire *wire;
    byte *buffer;
    byte bufferSize;
};

#endif // I2C_h