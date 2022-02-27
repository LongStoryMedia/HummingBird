#ifndef PID_H
#define PID_H

class Pid
{
private:
    float integratorLimit;
    uint16_t integratorThreashold;
    ScaledState desiredState;
    ScaledState prevDesiredState;
    Coefficients roll;
    Coefficients pitch;
    Coefficients yaw;
    CoefficientSet k;
    YPR out;
    YPR error;
    YPR prevError;
    YPR integral;
    YPR prevIntegral;
    YPR derivative;
    AccelGyro prevImu;
    void simpleAngle(AccelGyro imu);
    void cascadingAngle(AccelGyro imu);
    void simpleRate(AccelGyro imu);
    Commands commands;
    PropConfig propConfig;
    float errorAlt;
    float integralAlt;
    float derivativeAlt;
    float prevIntegralAlt;
    float mix(Prop prop);
    State lockAlt();

public:
    void init();
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
    void setDesiredState();
    Commands control(AccelGyro imu);

    enum mode
    {
        simpleAngleMode,
        cascadingAngleMode,
        simpleRateMode
    };
};

#endif