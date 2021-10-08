#ifndef PID_H
#define PID_H

class Pid
{
private:
    int16_t yawTarget;
    int16_t pitchTarget;
    int16_t rollTarget;
    uint8_t thrustTarget;
    float yaw;
    float pitch;
    float roll;
    float yawError;
    float pitchError;
    float rollError;
    float previousYawError;
    float integralYawError;
    float integralPitchError;
    float integralRollError;
    float previousIntegralYawError;
    float previousIntegralPitchError;
    float previousIntegralRollError;
    float integratorLimit;
    // coefficients
    float KpR; // proportional coefficient
    float KiR; // integral coefficient
    float KdR; // derivative coefficient
    float KpP; // proportional coefficient
    float KiP; // integral coefficient
    float KdP; // derivative coefficient
    float KpY; // proportional coefficient
    float KiY; // integral coefficient
    float KdY; // derivative coefficient

public:
    void setTargets(int16_t yaw, int16_t pitch, int16_t roll, uint16_t thrust);
    void processTick(float yaw, float pitch, float roll, float gx, float gy, float gz, uint32_t dt);
    void setCoefficients(float KpRoll, float KpPitch, float KpYaw, float KiRoll, float KiPitch, float KiYaw, float KdRoll, float KdPitch, float KdYaw, float iLimit);
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
};

#endif