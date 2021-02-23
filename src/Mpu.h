#ifndef MPU_H
#define MPU_H

#include <Wire.h>

class Mpu
{
private:
    uint16_t space[7];
    const int _mpu = 0x68;

public:
    void wire();
    void setSpace();
    uint16_t *getSpace();
    enum MPU
    {
        AcX,
        AcY,
        AcZ,
        Tmp,
        GyX,
        GyY,
        GyZ
    };
};

#endif