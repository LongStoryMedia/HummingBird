#ifndef ALT_H
#define ALT_H

class Alt
{
private:
#if defined(USE_BMP390)
    BMP390 baro;
#else
    MPL3115A2 baro;
#endif
#if defined(USE_USS_ALT)
    HCSR04 ussAlt = HCSR04(USS_DN_PIN, USS_TRIG_PIN);
    uint32_t prevUssDistance;
#endif
    float prevAlt;

    template <class T>
    void filterAltData(T prevVal, T currVal, float filterParam);

public:
    Alt();
    enum lockState
    {
        unlocked,
        locked
    };
    void init(TwoWire *wire);
    bool setAltLock(bool locked);
    float getAlt();
    uint32_t getAltUss();
    float lockedAlt;
    uint32_t lockedThrust;
    lockState altLocked;
    float alt;
#if defined(USE_USS_ALT)
    uint32_t ussDistance;
#endif
};

#endif