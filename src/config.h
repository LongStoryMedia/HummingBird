#ifndef CONFIG_H
#define CONFIG_H

// Macro for adding quotes
#define STRINGIFY(X) STRINGIFY2(X)
#define STRINGIFY2(X) #X

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //
//========================================================================================================================//

#if defined(PARAMS)
#include STRINGIFY(PARAMS)
#else
#include "params/default.h"
#endif
//========================================================================================================================//
#include "types.h"

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

#if defined(USE_PWM)
#if defined(ESP32)
#include <ESP32Servo.h>
#elif defined(TEENSY)
#include <PWMServo.h> //commanding any extra actuators, installed with teensyduino installer
#define Servo PWMServo
#else
#include <Servo.h>
#endif
#endif

#if defined(USE_MPL3115A2)
#include "Alt.h"
#endif

#include "Imu.h"
#include "Rx.h"
#include "Esc.h"
#include "Pid.h"
#include "Madgwick.h"

//========================================================================================================================//
//                                                 GLOBALS                                                                //
//========================================================================================================================//

extern uint32_t print_counter,
    serial_counter;
extern uint32_t blink_counter, blink_delay;
extern bool blinkAlternate;
extern Timer timer;
extern AccelGyro ag;
extern AccelGyro agPrev;
extern AccelGyro agError;
extern AccelGyro agImu;
extern AccelGyro agImuPrev;
extern Quaternion q;
extern Filter filter;
extern State packet;
extern PropConfig propConfig;
extern Rx rx;
extern Esc esc;
extern Pid pid;
extern Imu imu;
#if defined(USE_MPL3115A2)
extern Alt alt;
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
#define GYRO_FS_SEL_250 imu.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500 imu.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000 imu.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000 imu.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2 imu.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4 imu.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8 imu.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16 imu.ACCEL_RANGE_16G
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

// #define ESC_PROGRAM_MODE

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