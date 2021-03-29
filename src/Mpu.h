#ifndef MPU_LOCAL_H
#define MPU_LOCAL_H

class Mpu
{
private:
#if ACCGYROEXTERN
    MPU6050 mpu;

    VectorFloat gravity; // [x, y, z]            gravity vector
    Quaternion q;        // [w, x, y, z]         quaternion container
#endif

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

    void calibrate();
    void setSpace();
    enum YPR
    {
        yaw,
        pitch,
        roll
    };
};
#endif