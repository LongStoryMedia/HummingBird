#ifndef CONFIG_H
#define CONFIG_H
/* motor layout
        -
      pitch
     |1| |3|
       \ /
     + roll -
       / \
     |2| |4|
      pitch
        +
*/

#define DEBUG false
#define DEBUG_HZ false
#define ESC_TEST false
#define IMU_MPU6050 true
#define IMU_MPU9250 false

#define KP_ROLL 0.55f
#define KI_ROLL 0.15f
#define KD_ROLL 0.0001f

#define KP_PITCH 0.55f
#define KI_PITCH 0.15f
#define KD_PITCH 0.001f

#define KP_YAW 1.50
#define KI_YAW 0.25f
#define KD_YAW 0.000015f

#define I_LIMIT 25.0f

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#if TEENSY40
#include <PWMServo.h>
#else
#include <Servo.h>
#endif
#if INTEGRATED_BLE
#include <ArduinoBLE.h>
#endif

#if defined ACCGYROEXTERN
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#if IMU_MPU9250
#include <MPU9250.h>
#else
#include <I2Cdev.h>
#include <MPU6050.h>
#endif
#endif

#if LSM9DS1
#include <Arduino_LSM9DS1.h>
#endif
#if DEBUG
#endif

typedef void (*rateLimit)(int);

#include "Mpu.h"
#include "Esc.h"
#include "Pid.h"
#include "Rx.h"

#endif