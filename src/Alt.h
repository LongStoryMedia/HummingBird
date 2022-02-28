#ifndef ALT_H
#define ALT_H
#include "MPL3115A2.h"

class Alt
{
private:
    MPL3115A2 baro;
    float alt;
    float prevAlt;
    const float filterParam = 0.0008;

public:
    Alt();
    enum lockState
    {
        unlocked,
        locked
    };
    void init();
    bool setAltLock(bool locked);
    float getAlt();
    void altCheck();
    float lockedAlt;
    uint32_t lockedThrust;
    lockState altLocked;
};

#endif