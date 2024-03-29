#ifndef _TIMER_LOCAL_H_
#define _TIMER_LOCAL_H_

class Timer
{
public:
    Timer(uint16_t hz, uint16_t priority);

    void regulate(void (*func)());
    void update();
    void setClock(Timer *innerTimer = NULL);
    static float hzToUs(int speed);

    float delta;
    uint32_t now;
    uint32_t prev;
    uint32_t loopRate;
    uint32_t timeRemaining;
    uint32_t totalLoopTime;
    bool updated;
    float actualRate;
    uint8_t lowRateCount;

private:
    IntervalTimer timer;
};

#endif