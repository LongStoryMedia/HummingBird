#include "config.h"
#include "Alt.h"

void Alt::init()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    bool baroInit = baro.begin(&Wire1);
    if (!baroInit)
    {
        Serial.println("baro failed to start - check your wiring");
        while (1)
            ;
    }
    // use to set sea level pressure for current location
    // this is needed for accurate altitude measurement
    // STD SLP = 1013.26 hPa
    baro.setSeaPressure(1013.26);
}

bool Alt::setAltLock(bool locked)
{
    altLocked = locked;
    if (altLocked)
    {
        lockedAlt = baro.getAltitude();
    }
    return altLocked;
}

float Alt::getAlt()
{
    return baro.getAltitude();
}