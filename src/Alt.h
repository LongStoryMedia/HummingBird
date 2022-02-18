#ifndef ALT_H
#define ALT_H
#include "MPL3115A2.h"

class Alt
{
private:
    MPL3115A2 baro;
    bool altLocked;
    float alt;
    float prevAlt;
    const float filterParam = 0.00008;

public:
    void init();
    bool setAltLock(bool locked);
    float getAlt();
    float lockedAlt;
};

#endif