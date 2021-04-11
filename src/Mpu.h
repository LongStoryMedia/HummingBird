#ifndef MPU_LOCAL_H
#define MPU_LOCAL_H

typedef struct
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float yaw;
    float pitch;
    float roll;
} Orientation;

class Mpu
{
private:
#if defined ACCGYROEXTERN
#if defined IMU_MPU9250
    MPU9250 mpu;
#else
    MPU6050 mpu;

    VectorFloat gravity; // [x, y, z]            gravity vector
    Quaternion q;        // [w, x, y, z]         quaternion container
#endif
#endif
    void getError();

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    Orientation orientation;

    void correct();

public:
    float rawYpr[3];

    static volatile bool mpuInterrupt; // indicates whether MPU interrupt pin has gone high
    static void dmpDataReady()
    {
        mpuInterrupt = true;
    }

    void calibrate();
    Orientation getOrientation();
    float AccErrorX;
    float AccErrorY;
    float AccErrorZ;
    float GyroErrorX;
    float GyroErrorY;
    float GyroErrorZ;
};
#endif