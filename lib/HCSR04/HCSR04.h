#ifndef _HCSR04_H
#define _HCSR04_h

typedef void (*vfp)();

class HCSR04
{
public:
    HCSR04(uint8_t echo, uint8_t trig);
    HCSR04(uint8_t echoTrig);
    void start();
    void stop();
    void init();
    uint32_t ping();
    static HCSR04 *instance()
    {
        return self;
    }
    vfp getInterrupt();
    uint8_t echoPin;
    uint8_t trigPin;

private:
    volatile uint32_t sent;
    volatile uint32_t recieved;
    volatile bool isActive;
    static HCSR04 *self;
    volatile uint32_t distance;
};

#endif