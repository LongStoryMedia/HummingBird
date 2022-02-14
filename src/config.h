#ifndef CONFIG_H
#define CONFIG_H

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

//========================================================================================================================//

#include <Arduino.h>
#include <Wire.h> //I2c communication
#include <SPI.h>  //SPI communication

#include "nRF24L01.h"
#include "RF24.h"

#if defined(IMU_MPU6050)
#include "MPU6050.h"
#elif defined(IMU_MPU9250)
#include "MPU9250.h"
#elif defined(IMU_LSM9DS1)
#include "LSM9DS1.h"
#endif

//========================================================================================================================//

// Setup gyro and accel full scale value selection and scale factor

#if defined IMU_MPU6050
#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16
#elif defined IMU_MPU9250
#define GYRO_FS_SEL_250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16 mpu9250.ACCEL_RANGE_16G
#endif

#if defined IMU_LSM9DS1
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 1
#elif defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined IMU_LSM9DS1
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 1
#elif defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

#ifdef ESP32
#include <ESP32Servo.h>
#elif defined TEENSY
#include <PWMServo.h> //commanding any extra actuators, installed with teensyduino installer
#define Servo PWMServo
#else
#include <Servo.h>
#endif

#define PID_MODE 0

// Controller parameters (take note of defaults before modifying!):
#define I_LIMIT 20.0f         // Integrator saturation level, mostly for safety (default 25.0)
#define MAX_ROLL 30.0f        // Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_PITCH 30.0f       // Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_YAW 160.0f        // Max yaw rate in deg/sec
#define KP_ROLL_ANGLE 0.3f    // Roll P-gain - angle mode
#define KI_ROLL_ANGLE 0.35f   // Roll I-gain - angle mode
#define KD_ROLL_ANGLE 0.035f  // Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_ROLL 0.9f      // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_PITCH_ANGLE 0.3f   // Pitch P-gain - angle mode
#define KI_PITCH_ANGLE 0.35f  // Pitch I-gain - angle mode
#define KD_PITCH_ANGLE 0.035f // Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_PITCH 0.9f     // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_ROLL_RATE 0.15f    // Roll P-gain - rate mode
#define KI_ROLL_RATE 0.02f    // Roll I-gain - rate mode
#define KD_ROLL_RATE 0.0002f  // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_PITCH_RATE 0.15f   // Pitch P-gain - rate mode
#define KI_PITCH_RATE 0.2f    // Pitch I-gain - rate mode
#define KD_PITCH_RATE 0.0002f // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_YAW 0.08f          // Yaw P-gain
#define KI_YAW 0.2f           // Yaw I-gain
#define KD_YAW 0.000025f      // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

#define M1_PIN 4
#define M2_PIN 2
#define M3_PIN 3
#define M4_PIN 5

//========================================================================================================================//
//                                                      PROTOTYPES                                                        //
//========================================================================================================================//

void setupBlink(int numBlinks, int upTime, int downTime);
void loopBlink();
void loopRate(int freq);
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
        this->m1 = c;
        this->m2 = c;
        this->m3 = c;
        this->m4 = c;
    }
} Commands;

struct Timer
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
};

struct YPR
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
};

struct AccelGyro
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
};

enum pidMode
{
    simpleAngle,
    cascadingAngle,
    simpleRate
};
extern Timer timer;
extern AccelGyro ag;
extern AccelGyro agPrev;
extern AccelGyro agError;
extern AccelGyro agImu;
extern AccelGyro agImuPrev;
extern Quaternion q;
extern Filter filter;

float invSqrt(float x);
void setupBlink(int numBlinks, int upTime, int downTime);
void loopBlink();
void loopRate(int freq);

#include "Imu.h"
#include "Rx.h"
#include "Esc.h"
#include "Pid.h"
#include "Madgwick.h"

#endif