#ifndef IMU_LOCAL_H
#define IMU_LOCAL_H

class Imu
{
private:
#if defined ACCGYROEXTERN
#if IMU_MPU9250
    MPU9250 mpu;
#else
    MPU6050 mpu;
#endif
#endif

    // AccelGyro ag;
    // AccelGyro agError;

public:
    void calibrate();
    void init();
    void getImu();
};
#endif