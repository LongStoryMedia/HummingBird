#ifndef _I_BARO_
#define _I_BARO_

#include "Arduino.h"
#include "I2Cdev.h"

class IBaro
{
public:
    virtual ~IBaro(){};
    virtual float read() = 0;
    virtual void init(int basis, unsigned long clockspeed, TwoWire *wire) = 0;

protected:
    I2Cdev *i2c;
    bool oneShot();
    uint32_t lastUpdateTime;
    uint32_t clockSpeed;
    float alt;
};

#endif