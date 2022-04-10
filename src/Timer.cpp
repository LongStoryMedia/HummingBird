#include "config.h"

Timer::Timer(uint16_t hz)
{
    loopRate = hz;
    totalLoopTime = Timer::hzToUs(hz);
}

void Timer::setClock()
{
    uint32_t microsPassed = micros() - now;
    if (totalLoopTime > microsPassed)
    {
        timeRemaining = totalLoopTime - microsPassed;
    }
    else
    {
        updated = false;
        timeRemaining = totalLoopTime;
    }
}

void Timer::regulate(void (*func)())
{
    // DESCRIPTION: waits in loop until totalLoopTime has passed. Maintains freq.
    // if (Timer::hzToUs(delta * 1000000) < loopRate - 5) // a little grace for loss of precision
    // {
    //     Serial.print(F("loop rate is below "));
    //     Serial.print(loopRate);
    //     Serial.print(F("Hz. Current rate is "));
    //     Serial.print(Timer::hzToUs(delta * 1000000));
    //     Serial.println(F("Hz"));
    // };

    // Sit in loop until appropriate time has passed
    while (updated)
    {
        if (func != NULL)
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