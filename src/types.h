#ifndef LOCAL_TYPES_H
#define LOCAL_TYPES_H

#include <Arduino.h>

typedef struct
{
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint16_t thrust;
    uint8_t lockAlt;
} State;

typedef struct
{
    float yaw;
    float pitch;
    float roll;
    float thrust;
    float alt;
} ScaledState;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;
} Coefficients;

typedef struct
{
    Coefficients yaw;
    Coefficients pitch;
    Coefficients roll;
} CoefficientSet;

typedef struct
{
    float madgwick; // Madgwick filter parameter
    float accel;    // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float gyro;     // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    float mag;      // Magnetometer LP filter parameter
} Filter;

typedef struct
{
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

typedef struct
{
    int m1;
    int m2;
    int m3;
    int m4;
    void operator=(uint32_t c)
    {
        m1 = c;
        m2 = c;
        m3 = c;
        m4 = c;
    }
    void operator-=(uint32_t c)
    {
        m1 -= c;
        m2 -= c;
        m3 -= c;
        m4 -= c;
    }
} Commands;

typedef struct YPR
{
    float yaw;
    float pitch;
    float roll;
    YPR operator+=(YPR ypr)
    {
        yaw += ypr.yaw;
        pitch += ypr.pitch;
        roll += ypr.roll;
        return *this;
    }
    YPR operator-=(YPR ypr)
    {
        yaw -= ypr.yaw;
        pitch -= ypr.pitch;
        roll -= ypr.roll;
        return *this;
    }
    YPR operator*=(YPR ypr)
    {
        yaw *= ypr.yaw;
        pitch *= ypr.pitch;
        roll *= ypr.roll;
        return *this;
    }
    YPR operator/=(YPR ypr)
    {
        yaw /= ypr.yaw;
        pitch /= ypr.pitch;
        roll /= ypr.roll;
        return *this;
    }
    YPR operator/=(float yprMultiplier)
    {
        yaw /= yprMultiplier;
        pitch /= yprMultiplier;
        roll /= yprMultiplier;
        return *this;
    }
    YPR operator*=(float yprMultiplier)
    {
        yaw *= yprMultiplier;
        pitch *= yprMultiplier;
        roll *= yprMultiplier;
        return *this;
    }
    YPR operator+(YPR ypr)
    {
        yaw += ypr.yaw;
        pitch += ypr.pitch;
        roll += ypr.roll;
        return *this;
    }
    YPR operator-(YPR ypr)
    {
        yaw -= ypr.yaw;
        pitch -= ypr.pitch;
        roll -= ypr.roll;
        return *this;
    }
    YPR operator*(YPR ypr)
    {
        yaw *= ypr.yaw;
        pitch *= ypr.pitch;
        roll *= ypr.roll;
        return *this;
    }
    YPR operator*(float yprMultiplier)
    {
        yaw *= yprMultiplier;
        pitch *= yprMultiplier;
        roll *= yprMultiplier;
        return *this;
    }
    YPR operator/(YPR ypr)
    {
        yaw /= ypr.yaw;
        pitch /= ypr.pitch;
        roll /= ypr.roll;
        return *this;
    }
    YPR operator/(float yprMultiplier)
    {
        yaw /= yprMultiplier;
        pitch /= yprMultiplier;
        roll /= yprMultiplier;
        return *this;
    }
} YPR;

typedef struct AccelGyro
{
    YPR accel;
    YPR gyro;
    YPR mag;
    AccelGyro operator+=(AccelGyro ag)
    {
        accel += ag.accel;
        gyro += ag.gyro;
        mag += ag.mag;
        return *this;
    }
    AccelGyro operator-=(AccelGyro ag)
    {
        accel -= ag.accel;
        gyro -= ag.gyro;
        mag -= ag.mag;
        return *this;
    }
    AccelGyro operator*=(AccelGyro ag)
    {
        accel *= ag.accel;
        gyro *= ag.gyro;
        mag *= ag.mag;
        return *this;
    }
    AccelGyro operator/=(AccelGyro ag)
    {
        accel /= ag.accel;
        gyro /= ag.gyro;
        mag /= ag.mag;
        return *this;
    }
    AccelGyro operator*=(float yprMultiplier)
    {
        accel *= yprMultiplier;
        gyro *= yprMultiplier;
        mag *= yprMultiplier;
        return *this;
    }
    AccelGyro operator/=(float yprMultiplier)
    {
        accel /= yprMultiplier;
        gyro /= yprMultiplier;
        mag /= yprMultiplier;
        return *this;
    }
    AccelGyro operator+(AccelGyro ag)
    {
        accel += ag.accel;
        gyro += ag.gyro;
        mag += ag.mag;
        return *this;
    }
    AccelGyro operator-(AccelGyro ag)
    {
        accel -= ag.accel;
        gyro -= ag.gyro;
        mag -= ag.mag;
        return *this;
    }
    AccelGyro operator*(AccelGyro ag)
    {
        accel *= ag.accel;
        gyro *= ag.gyro;
        mag *= ag.mag;
        return *this;
    }
    AccelGyro operator/(AccelGyro ag)
    {
        accel /= ag.accel;
        gyro /= ag.gyro;
        mag /= ag.mag;
        return *this;
    }
    AccelGyro operator/(float yprMultiplier)
    {
        accel /= yprMultiplier;
        gyro /= yprMultiplier;
        mag /= yprMultiplier;
        return *this;
    }
    AccelGyro operator*(float yprMultiplier)
    {
        accel *= yprMultiplier;
        gyro *= yprMultiplier;
        mag *= yprMultiplier;
        return *this;
    }
} AccelGyro;

typedef struct
{
    enum rotation
    {
        counterClockwise = -1,
        _,
        clockwise
    };
    enum axis
    {
        negative = -1,
        __,
        positive
    };
    rotation rotation;
    axis xAxis;
    axis yAxis;
} Prop;

typedef struct
{
    Prop p1{p1._, p1.positive, p1.negative};
    Prop p2{p2._, p2.negative, p2.negative};
    Prop p3{p3._, p3.negative, p3.positive};
    Prop p4{p4._, p4.positive, p4.positive};
} PropConfig;

enum configType
{
    x1,
    x2
};

enum imuOrientation
{
    upForward,
    upBackward,
    downForward,
    downBackward
};

typedef struct
{
    uint8_t frontLeft;
    uint8_t frontRight;
    uint8_t left;
    uint8_t right;
    uint8_t rear;
    bool hasObstacles()
    {
        return frontLeft + frontRight + left + right + rear < 5;
    }
    void clear()
    {
        frontLeft = 0;
        frontRight = 0;
        left = 0;
        right = 0;
        rear = 0;
    }
} obstacles;

//========================================================================================================================//
//                                                      PROTOTYPES                                                        //
//========================================================================================================================//

float invSqrt(float x);
void setupBlink(int numBlinks, int upTime, int downTime);
void loopBlink();
void loopRate(int freq);
template <class T>
void debug(T data);
#endif