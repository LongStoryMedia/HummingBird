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

public:
    void init();
    uint16_t r1;
    uint16_t r2;
    uint16_t r3;
    uint16_t r4;
    void setDesiredState(State packet);
    Commands control(AccelGyro imu);
};

#endif