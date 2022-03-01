#include "config.h"
#if !defined(USE_MPL3115A2)
#include "MPL3115A2.h"
#include "Alt.h"
#endif

Alt::Alt()
{
    lockedAlt = 0.0;
    lockedThrust = 0;
}

void Alt::init(TwoWire *_wire = &Wire)
{
    // baro = MPL3115A2(_wire);
    baro.init();
}

void Alt::altCheck()
{
    if (altLocked != (lockState)packet.lockAlt)
    {
        altLocked = (lockState)packet.lockAlt;
    }

    if (altLocked == locked && lockedAlt == 0.0)
    {
        lockedAlt = baro.read().smooth;
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
    MPL3115A2::_baro baroData = baro.read();
    alt = baroData.smooth;
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
