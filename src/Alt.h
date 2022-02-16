#ifndef ALT_H
#define ALT_H
#include "MPL3115A2.h"

class Alt
{
private:
    MPL3115A2 baro;
    bool altLocked;

public:
    void init();
    bool setAltLock(bool locked);
    float lockedAlt;
};

#endif