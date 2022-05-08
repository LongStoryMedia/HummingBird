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
    baro.init(300, radioTimer.totalLoopTime, wire);
#if defined(USE_USS_ALT)
    ussAlt.init();
    attachInterrupt(
        digitalPinToInterrupt(ussAlt.echoPin),
        ussAlt.getInterrupt(),
        CHANGE);
    ussAlt.start();
#endif
}

float Alt::getAlt()
{
    prevAlt = alt;
    // noInterrupts();
    alt = baro.read();
    // interrupts();

    filterAltData<float>(prevAlt, alt, 0.0008);

    return alt;
}

uint32_t Alt::getAltUss()
{
#if defined(USE_USS_ALT)
    prevUssDistance = ussDistance;
    ussDistance = ussAlt.ping();

    filterAltData<uint32_t>(prevUssDistance, ussDistance, 80.5);
    Serial.print("Distance: ");
    Serial.print(ussDistance);
    Serial.println(" cm");

    return ussDistance;
#endif
}

template <class T>
void Alt::filterAltData(T prevVal, T currVal, float filterParam)
{
    if (prevVal == 0)
    {
        prevVal = currVal;
    }

    // only integrate on change
    if (currVal != prevVal)
    {
        float err = currVal - prevVal;
        // LP filter alt data
        currVal = (1.0 - filterParam) * prevVal + filterParam * currVal;
        currVal += err * altTimer.delta;
    }
}