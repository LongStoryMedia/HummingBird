#ifndef PID_H
#define PID_H

class Pid
{
private:
    float integratorLimit;
    uint16_t integratorThreashold;
    CoefficientSet kAngle;
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
    void angleLoop(const AccelGyro &imu);
    void rateLoop(const AccelGyro &imu);
    Commands commands;
    float errorAlt;
    float integralAlt;
    float derivativeAlt;
    float prevIntegralAlt;
    float lockedDesiredThrust;
    float errorThrust;
    float mix(Prop prop);
    void integrateAlt(Input packet);
    bool isPreTakeoff();

public:
    void init();
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
    void setDesiredState(Input packet);
    Commands control(const AccelGyro &imu);

    enum mode
    {
        simpleAngleMode,
        cascadingMode,
        simpleRateMode
    };
};

#endif