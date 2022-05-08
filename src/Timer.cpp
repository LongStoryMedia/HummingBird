#include "config.h"

Timer::Timer(uint16_t hz, uint16_t priority)
{
    loopRate = hz;
    totalLoopTime = Timer::hzToUs(hz);
    timer.priority(priority);
}

bool Timer::timesUp()
{
    return totalLoopTime <= (micros() - now);
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
    if (timesUp())
    {
        Serial.print(F("loop rate is below "));
        Serial.print(loopRate);
        Serial.print(F("Hz. Current rate is "));
        Serial.println(Timer::hzToUs(delta * 1000000.0));
    };
}

float Timer::hzToUs(int speed)
{
    return 1.0 / speed * 1000000.0;
}