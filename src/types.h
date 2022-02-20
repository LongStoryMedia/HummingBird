#ifndef LOCAL_TYPES_H
#define LOCAL_TYPES_H

#include <Arduino.h>

typedef struct
{
    int32_t yaw;
    int32_t pitch;
    int32_t roll;
    int32_t thrust;
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
} Commands;

typedef struct
{
    float delta;
    uint32_t now;
    uint32_t prev;
    void update()
    {
        prev = now;
        now = micros();
        delta = (now - prev) / 1000000.0;
    }
} Timer;

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
    int8_t rotation;
    int8_t xAxis;
    int8_t yAxis;
} Prop;

typedef struct
{
    Prop p1{p1._, p1.positive, p1.negative};
    Prop p2{p2._, p2.negative, p2.negative};
    Prop p3{p3._, p3.negative, p3.positive};
    Prop p4{p4._, p4.positive, p4.positive};
    float mix(YPR out, Prop prop)
    {
        return (float)(prop.xAxis * out.roll) + (float)(prop.yAxis * out.pitch) + (float)(prop.rotation * out.yaw);
    }
} PropConfig;

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