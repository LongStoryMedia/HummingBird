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
    baro.init(300, timerOl.totalLoopTime, wire);
}

float Alt::getAlt()
{
    prevAlt = alt;
    noInterrupts();
    alt = baro.read();
    interrupts();

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
    // if (timer.now - lastUpdate > Timer::hzToUs(timer.loopRate / 20))
    // {
    //     alt = baro.read();
    // }

    // only integrate on change
    if (alt != prevAlt)
    {
        float err = alt - prevAlt;
        // LP filter alt data
        alt = (1.0 - filterParam) * prevAlt + filterParam * alt;
        alt += err * timerOl.delta;
        realAlt = alt;
        lastUpdate = micros();
    }

    return alt;
}
