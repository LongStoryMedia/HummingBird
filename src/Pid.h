#ifndef PID_H
#define PID_H

#include <Arduino.h>

// typedef uint16_t (*processControlOutput)(float proportionalTerm, float integralTerm);

class Pid
{
private:
    float pitch;
    float roll;
    float pitchError;
    float rollError;
    float accumulatedPitchError;
    float accumulatedRollError;
    float integralTerm;
    float proportionalTerm;
    uint16_t deltaTime;
    unsigned long time;
    uint16_t r1correct();
    uint16_t r2correct();
    uint16_t r3correct();
    uint16_t r4correct();

public:
    // coefficients
    float Kp; // proportional coefficient
    float Ki; // integral coefficient
    void processTick(float pitchTarget, float rollTarget);
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
};

// i imagine one class for each rotor,
// and the `process` property replaced with two properties -
// one for pitch, and another for roll
// then `processTick` would either be run against both
// and the outputs combined, mixed, or averaged or something
// or it would take both into account

#endif