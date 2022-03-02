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

void Alt::init()
{
    // this altitude must be known (or provided by GPS etc.)
    // altitude from mn (https://whatismyelevation.com/)
    baro.init(300, hzToUs(2000), &Wire1);
}

void Alt::altCheck()
{
    // this should be called once per loop
    // and no more - in order to maintain proper frequency
    alt = getAlt();

    if (altLocked != (lockState)packet.lockAlt)
    {
        if (packet.lockAlt == packet.locked)
        {
            lockedAlt = alt;
            lockedThrust = packet.thrust;
            Serial.print(F("locking alt at alt "));
            Serial.print(lockedAlt);
            Serial.print(F(" and thrust "));
            Serial.println(lockedThrust);
        }
        altLocked = (lockState)packet.lockAlt;
    }
}

float Alt::getAlt()
{
    prevAlt = alt;
    alt = baro.read().smooth;

    if (prevAlt == 0)
    {
        prevAlt = alt;
    }

    // filter noise
    if (abs(alt - prevAlt) > 100)
    {
        alt = prevAlt;
    }

    // try to get an update if we haven't seen one in a while
    if (timer.now - lastUpdate > hzToUs(timer.loopRate / 10))
    {
        alt = baro.read().raw;
    }

    // only integrate on change
    if (alt != prevAlt)
    {
        float err = alt - prevAlt;
        // LP filter alt data
        alt = (1.0 - filterParam) * prevAlt + filterParam * alt;
        alt += err * timer.delta;
        lastUpdate = micros();
    }

    return alt;
}
