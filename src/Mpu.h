#ifndef MPU_LOCAL_H
#define MPU_LOCAL_H

#include <ArduinoJson.h>
const size_t DOC_SIZE = JSON_OBJECT_SIZE(6);
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

class Mpu
{
private:
    MPU6050 mpu;

    VectorFloat gravity; // [x, y, z]            gravity vector
    Quaternion q;        // [w, x, y, z]         quaternion container

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
public:
    float rawYpr[3];
    int16_t ypr[3]; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

    static volatile bool mpuInterrupt; // indicates whether MPU interrupt pin has gone high
    static void dmpDataReady()
    {
        mpuInterrupt = true;
    }
    void flush();

    void calibrate();
    void setSpace();
    enum YPR
    {
        yaw,
        pitch,
        roll
    };
    StaticJsonDocument<DOC_SIZE> doc;
};
#endif