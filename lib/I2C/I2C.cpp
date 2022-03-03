#include "I2C.h"

I2C::I2C(byte addr, byte size, TwoWire *theWire = &Wire)
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

bool I2C::write(byte regAddr, byte value)
{
    wire->beginTransmission(address);
    wire->write(regAddr);
    wire->write(value);
    if (wire->endTransmission(true) == 0)
    {
        return true;
    }
    return false;
}

bool I2C::write(byte regAddr, byte *value, byte length = 0)
{
    wire->beginTransmission(address);
    wire->write(regAddr);
    for (uint8_t i = 0; i < length; i++)
    {
        Wire.write((uint8_t)value[i]);
    }

    return wire->endTransmission(true) == 0;
}

byte I2C::getBufferByte(byte idx)
{
    return buffer[idx];
}

void I2C::clearBuffer()
{
    delete[] buffer;
    buffer = new byte[bufferSize];
}

byte I2C::readBits(byte regAddr, byte bitStart, byte length, unsigned short timeout = 0)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    byte count, b;
    if ((count = readByte(regAddr, timeout)) != 0)
    {
        byte mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *buffer = b;
    }
    return count;
}

byte I2C::readBit(byte regAddr, byte bitNum, unsigned long timeout = 0)
{
    uint8_t b;
    uint8_t count = readByte(regAddr, timeout);
    *buffer = b & (1 << bitNum);
    return count;
}

byte I2C::readByte(byte regAddr, unsigned short timeout = 0)
{
    return readSomeBytes(regAddr, 1, timeout);
}

byte I2C::readSomeBytes(byte regAddr, byte length, unsigned short timeout = 0)
{
    byte count = 0;
    unsigned long t1 = millis();

    for (byte k = 0; k < length; k += min((int)length, BUFFER_LENGTH))
    {
        wire->beginTransmission(address);
        wire->write(regAddr);
        wire->endTransmission();
        wire->beginTransmission(address);
        wire->requestFrom(address, (byte)min(length - k, BUFFER_LENGTH));

        while (wire->available() && (timeout == 0 || millis() - t1 < timeout))
        {
            buffer[count++] = wire->read();
        }
    }

    // check for timeout
    if (timeout > 0 && millis() - t1 >= timeout && count < length)
    {
        count = -1; // timeout
    }

    return count;
}

bool I2C::writeBits(byte regAddr, byte bitStart, byte length, byte data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;
    if (readByte(regAddr) != 0)
    {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask;                     // zero all non-important bits in data
        b &= ~(mask);                     // zero all important bits in existing byte
        b |= data;                        // combine data with existing byte
        return write(regAddr, b);
    }
    else
    {
        return false;
    }
}

bool I2C::writeBit(byte regAddr, byte bitNum, byte data)
{
    uint8_t b;
    readByte(regAddr);
    *buffer = (data != 0) ? (*buffer | (1 << bitNum)) : (*buffer & ~(1 << bitNum));
    return write(regAddr, *buffer);
}