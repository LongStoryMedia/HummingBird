#ifndef PID_H
#define PID_H

#include <Arduino.h>

class Pid
{
private:
    float pitch;
    float roll;
    float pitchError;
    float rollError;
    uint16_t deltaTime;
    unsigned long time;
    // coefficients
    float Kp; // proportional coefficient
    float Ki; // integral coefficient

public:
    void processTick(int16_t pitch, int16_t roll, int16_t pitchTarget, int16_t rollTarget, uint16_t thrust);
    void setCoefficients(float proportionalCoefficient, float integralCoefficient);
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
};

#endif