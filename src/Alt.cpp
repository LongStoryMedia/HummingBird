#include "config.h"
#if !defined(USE_ALT)
#include "MPL3115A2.h"
#include "Alt.h"
#endif

Alt::Alt()
{
    lockedAlt = 0.0;
    lockedThrust = 0;
}

void Alt::init(TwoWire *wire)
{
    // this altitude must be known (or provided by GPS etc.)
    // altitude from mn (https://whatismyelevation.com/)
    baro.init(300, hzToUs(200), wire);
}

void Alt::altCheck()
{
    // this should be called once per loop
    // and no more - in order to maintain proper frequency
    alt = getAlt();
    Serial.println(alt);

    if (altLocked != (lockState)packet.lockAlt)
    {
        if (packet.lockAlt == 1)
        {
            lockedAlt = alt;
            lockedThrust = packet.thrust;
            Serial.print(F("locking alt at alt "));
            Serial.print(lockedAlt);
            Serial.print(F(" and thrust "));
            Serial.println(lockedThrust);
            Serial.print(F("delta "));
            Serial.println(timer.delta);
        }
        altLocked = (lockState)packet.lockAlt;
    }
}

float Alt::getAlt()
{
    prevAlt = alt;
    alt = baro.read();

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
    if (timer.now - lastUpdate > hzToUs(timer.loopRate / 20))
    {
        alt = baro.read();
    }

    // only integrate on change
    if (alt != prevAlt)
    {
        float err = alt - prevAlt;
        // LP filter alt data
        alt = (1.0 - filterParam) * prevAlt + filterParam * alt;
        alt += err * timer.delta;
        realAlt = alt;
        lastUpdate = micros();
    }

    return alt;
}
