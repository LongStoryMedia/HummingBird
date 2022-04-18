#include "config.h"

Timer::Timer(uint16_t hz)
{
    loopRate = hz;
    totalLoopTime = Timer::hzToUs(hz);
}

bool Timer::timesUp()
{
    return totalLoopTime <= (micros() - now);
}

void Timer::setClock(Timer *innerTimer)
{
    if (!timesUp())
    {
        timeRemaining = totalLoopTime - micros() - now;
        if (innerTimer->totalLoopTime > timeRemaining)
        {
            while (totalLoopTime - micros() - now)
                ;

            updated = false;
            timeRemaining = totalLoopTime;
        }
    }
    else
    {
        updated = false;
        timeRemaining = totalLoopTime;
    }
}

void Timer::regulate(void (*func)(), Timer *innerTimer)
{
    // DESCRIPTION: waits in loop until totalLoopTime has passed. Maintains freq.
    delta = (micros() - now) / 1000000.0;
    if (timesUp())
    {
        Serial.print(F("loop rate is below "));
        Serial.print(loopRate);
        Serial.print(F("Hz. Current rate is "));
        Serial.print(Timer::hzToUs(delta * 1000000.0));
        Serial.print(F("Hz, delta is "));
        Serial.print(delta * 1000000.0);
        Serial.print(F("ms, and total loop time is "));
        Serial.println(totalLoopTime);
        updated = false;
        timeRemaining = totalLoopTime;
        return;
    };
    // Sit in loop until appropriate time has passed
    while (updated)
    {
        if (func != NULL && innerTimer != NULL)
        {
            func();
        }
        setClock();
    }
}

void Timer::update()
{
    if (!updated)
    {
        prev = now;
        now = micros();
        delta = (now - prev) / 1000000.0;
        updated = true;
    }
}

float Timer::hzToUs(int speed)
{
    return 1.0 / speed * 1000000.0;
}