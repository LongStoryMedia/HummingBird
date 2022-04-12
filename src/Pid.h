#ifndef PID_H
#define PID_H

class Pid
{
private:
    float integratorLimit;
    uint16_t integratorThreashold;
    ScaledState desiredState;
    ScaledState prevDesiredState;
    Coefficients rollAngle;
    Coefficients pitchAngle;
    Coefficients yawAngle;
    CoefficientSet kAngle;
    Coefficients rollRate;
    Coefficients pitchRate;
    Coefficients yawRate;
    CoefficientSet kRate;
    YPR out;
    YPR error;
    YPR prevError;
    YPR integral;
    YPR prevIntegral;
    YPR derivative;
    YPR ol;
    YPR integralOl;
    YPR prevIntegralOl;
    AccelGyro prevImu;
    void simpleAngle(AccelGyro imu);
    void cascadingAngle(AccelGyro imu);
    void simpleRate(AccelGyro imu);
    Commands commands;
    float errorAlt;
    float integralAlt;
    float derivativeAlt;
    float prevIntegralAlt;
    float lockedDesiredThrust;
    float errorThrust;
    float mix(Prop prop);
    void integrateAlt(State packet);

public:
    void init();
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
    void setDesiredState(State packet);
    Commands control(AccelGyro imu);

    enum mode
    {
        simpleAngleMode,
        cascadingMode,
        simpleRateMode
    };
};

#endif