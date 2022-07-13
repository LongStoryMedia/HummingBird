#include "config.h"

Timer::Timer(uint16_t hz, uint16_t priority)
{
    loopRate = hz;
    totalLoopTime = hzToUs(hz);
    timer.priority(priority);
    lowRateCount = 0;
}

void Timer::regulate(void (*func)())
{
    timer.begin(func, totalLoopTime);
}

void Timer::update()
{
    prev = now;
    now = micros();
    delta = (now - prev) / 1000000.0;
    actualRate = hzToUs(delta * 1000000.0);
    if (loopRate > actualRate)
    {
        lowRateCount++;
    }
    else
    {
        lowRateCount = 0;
    }

    if (lowRateCount > 1)
    {
        Serial.print(F("loop rate is below "));
        Serial.print(loopRate);
        Serial.print(F("Hz. Current rate is "));
        Serial.print(actualRate);
        Serial.println();
    }
}

float Timer::hzToUs(int speed)
{
    return 1.0 / speed * 1000000.0;
}