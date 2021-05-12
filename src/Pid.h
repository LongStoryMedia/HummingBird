#ifndef PID_H
#define PID_H

class Pid
{
private:
    int16_t yawTarget;
    int16_t pitchTarget;
    int16_t rollTarget;
    uint8_t thrustTarget;
    float pitch;
    float roll;
    float pitchError;
    float rollError;
    float integralPitchError;
    float integralRollError;
    float derivativePitchError;
    float derivativeRollError;
    uint16_t deltaTime;
    uint32_t time;
    // coefficients
    float Kp; // proportional coefficient
    float Ki; // integral coefficient
    float Kd; // derivative coefficient

public:
    void setTargets(int16_t pitch, int16_t roll, uint16_t thrust);
    void processTick(int16_t pitch, int16_t roll);
    void setCoefficients(float proportionalCoefficient, float integralCoefficient, float derivativeCoefficient);
    void setDerivatives(float rollDerivative, float pitchDerivative);
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
};

#endif