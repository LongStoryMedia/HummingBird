#include "Mpu.h"

void Mpu::wire()
{
    Wire.begin();
    Wire.beginTransmission(_mpu);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);
}

void Mpu::setSpace()
{
    Wire.beginTransmission(_mpu);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(_mpu, 12, true);
    space[AcX] = Wire.read() << 8 | Wire.read();
    space[AcY] = Wire.read() << 8 | Wire.read();
    space[AcZ] = Wire.read() << 8 | Wire.read();
    space[GyX] = Wire.read() << 8 | Wire.read();
    space[GyY] = Wire.read() << 8 | Wire.read();
    space[GyZ] = Wire.read() << 8 | Wire.read();
}

uint16_t *Mpu::getSpace()
{
    static uint16_t _space[7];
    for (int i = 0; i < 7; i++)
    {
        _space[i] = space[i];
    }
    return _space;
}