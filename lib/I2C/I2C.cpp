#include "I2C.h"

I2C::I2C(uint8_t addr, uint8_t size, TwoWire *theWire = &Wire)
{
    wire = theWire;
    bufferSize = size;
    address = addr;
    buffer = new byte[size];
}

byte I2C::read(byte regAddr)
{
    // This function reads one byte over I2C
    wire->beginTransmission(address);
    wire->write(regAddr);         // Address of CTRL_REG1
    wire->endTransmission(false); // Send data to I2C dev with option for a repeated start. Works in Arduino V1.0.1
    wire->requestFrom(address, 1);
    return wire->read();
}

void I2C::readBytes()
{
    byte i = 0;
    wire->beginTransmission(address);
    wire->write(0x01); // Address of CTRL_REG1
    wire->endTransmission(false);
    wire->requestFrom(address, bufferSize);
    while (wire->available())
    {
        buffer[i++] = wire->read();
    }
}

void I2C::write(byte regAddr, byte value)
{
    wire->beginTransmission(address);
    wire->write(regAddr);
    wire->write(value);
    wire->endTransmission(true);
}

byte I2C::getBufferByte(uint8_t idx)
{
    return buffer[idx];
}