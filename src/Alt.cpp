#include "config.h"
#include "Alt.h"

Alt::Alt()
{
    lockedAlt = 0.0;
    lockedThrust = 0;
}

void Alt::init()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    baro.begin(&Wire1);
    // use to set sea level pressure for current location
    // this is needed for accurate altitude measurement
    // STD SLP = 1013.26 hPa
    baro.setSeaPressure(1013.26);
    baro.setOversampleRate(7);
    baro.setModeAltimeter();
    baro.setModeActive();
}

void Alt::altCheck()
{
    if (altLocked != (lockState)packet.lockAlt)
    {
        altLocked = (lockState)packet.lockAlt;
    }

    if (altLocked == locked && lockedAlt == 0.0)
    {
        lockedAlt = getAlt();
        lockedThrust = packet.thrust;
        Serial.print(F("locking alt at alt "));
        Serial.print(lockedAlt);
        Serial.print(F(" and thrust "));
        Serial.println(lockedThrust);
    }

    if (altLocked == unlocked)
    {
        lockedAlt = 0.0;
        lockedThrust = 0;
    }
}

float Alt::getAlt()
{
    alt = baro.readAltitudeFt();
    // if (prevAlt == 0)
    // {
    //     prevAlt = baro.readAltitudeFt();
    // }
    // float err = alt - prevAlt;
    // // LP filter alt data
    // alt = (1.0 - filterParam) * prevAlt + filterParam * alt;
    // alt += err * timer.delta;
    // prevAlt = alt;
    return alt;
}
