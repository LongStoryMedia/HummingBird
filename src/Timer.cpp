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

void Timer::regulate(void (*func)(), uint16_t priority)
{
    timer.begin(func, totalLoopTime);
    timer.priority(priority);
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