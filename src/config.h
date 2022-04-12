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

#ifndef KL
#define KL 30.0f
#endif

#if defined(USE_MPL3115A2) || defined(USE_BMP390)
#define USE_ALT
#endif

//========================================================================================================================//
#include "Threading.h"

#include "types.h"

#include <Wire.h> //I2c communication
#include <SPI.h>  //SPI communication

#include "nRF24L01.h"
#include "RF24.h"
#include "MPU6050.h"

#if defined(USE_ALT)
#if defined(USE_MPL3115A2)
#include "MPL3115A2.h"
#elif defined(USE_BMP390)
#include "BMP390.h"
#endif
#include "Alt.h"
#endif

#if defined(USE_PROXIMITY_DETECTION)
#include "Proximity.h"
#endif

#include "Timer.h"
#include "Imu.h"
#include "Rx.h"
#include "Esc.h"
#include "Pid.h"
#include "Madgwick.h"

//========================================================================================================================//
//                                                 GLOBALS                                                                //
//========================================================================================================================//

extern uint32_t print_counter, serial_counter;
extern uint32_t blink_counter, blink_delay;
extern bool blinkAlternate;
extern Timer timer;
extern Timer radioTimer;
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
#if defined(USE_ALT)
extern Alt alt;
#endif
#if defined(USE_PROXIMITY_DETECTION)
extern Proximity proximity;
#endif

#if defined(USE_MULTISHOT)
#define COMMANDS_LOW 5
#define COMMANDS_HIGH 25
#elif defined(USE_ONESHOT_42)
#define COMMANDS_LOW 42
#define COMMANDS_HIGH 84
#elif defined(USE_PMW)
#define COMMANDS_LOW 0
#define COMMANDS_HIGH 180
#else // default is oneshot125
#define COMMANDS_LOW 125
#define COMMANDS_HIGH 250
#endif
//========================================================================================================================//

// Setup gyro and accel full scale value selection and scale factor

#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16

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

//========================================================================================================================//
//                                                      PROTOTYPES                                                        //
//========================================================================================================================//

float invSqrt(float x);
void setupBlink(int numBlinks, int upTime, int downTime);
void loopBlink();
void loopRate();
template <class T>
void debug(T data);

#endif