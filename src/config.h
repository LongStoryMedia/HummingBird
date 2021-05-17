#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG false
#define ESC_TEST false

#define KP 1.75f
#define KI 0.0001f
#define KD 0.0001f

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
#if defined IMU_MPU9250
#include <MPU9250.h>
#else
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#endif
#endif

#if LSM9DS1
#include <Arduino_LSM9DS1.h>
#endif
#if DEBUG
#endif

#include "Mpu.h"
#include "Esc.h"
#include "Pid.h"
#include "Rx.h"

#endif