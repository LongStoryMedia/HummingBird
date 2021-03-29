#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG false

#include <Arduino.h>
#include <ArduinoJson.h>
#include <Servo.h>
#if INTEGRATED_BLE
#include <ArduinoBLE.h>
#endif

#if ACCGYROEXTERN
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#endif

#if LSM9DS1
#include <Arduino_LSM9DS1.h>
#endif
#if DEBUG
#if !defined(Serial1)
#define Serial1 Serial
#endif
#endif

#include "Mpu.h"
#include "Esc.h"
#include "Pid.h"
#include "Rx.h"

#endif