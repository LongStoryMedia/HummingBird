#ifndef ALT_H
#define ALT_H

class Alt
{
private:
    MPL3115A2 baro;
    float prevAlt;
    float alt;
    const float filterParam = 0.0008;
    uint32_t lastUpdate;
    void setReal();

public:
    Alt();
    enum lockState
    {
        unlocked,
        locked
    };
    void init();
    bool setAltLock(bool locked);
    float getAlt();
    void altCheck();
    float lockedAlt;
    uint32_t lockedThrust;
    lockState altLocked;
    float realAlt;
};

#endif