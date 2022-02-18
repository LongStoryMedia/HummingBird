#include "config.h"
#include "Alt.h"

void Alt::init()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    baro.setOversampleRate(0);
    // use to set sea level pressure for current location
    // this is needed for accurate altitude measurement
    // STD SLP = 1013.26 hPa
    baro.setModeAltimeter();
    // baro.setModeActive();
}

bool Alt::setAltLock(bool locked)
{
    altLocked = locked;
    if (altLocked)
    {
        lockedAlt = baro.readAltitudeFt();
    }
    return altLocked;
}

float Alt::getAlt()
{
    // float norm, err, ig;
    alt = baro.readAltitudeFt();
    if (prevAlt == 0)
    {
        prevAlt = baro.readAltitudeFt();
    }
    // norm = invSqrt(alt * alt);
    // alt *= norm;
    float err = alt - prevAlt;
    // LP filter alt data
    alt = (1.0 - filterParam) * prevAlt + filterParam * alt;
    alt += err * timer.delta;
    prevAlt = alt;
    return alt;
}
