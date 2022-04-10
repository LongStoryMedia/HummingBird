#ifndef _TIMER_LOCAL_H_
#define _TIMER_LOCAL_H_

class Timer
{
public:
    Timer(uint16_t hz);

    void regulate(void (*func)() = NULL);
    void update();
    void setClock();
    static float hzToUs(int speed);

    float delta;
    uint32_t now;
    uint32_t prev;
    uint32_t loopRate;
    uint32_t timeRemaining;
    uint32_t totalLoopTime;
    bool updated;

private:
};

#endif