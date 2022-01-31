#ifndef IMU_LOCAL_H
#define IMU_LOCAL_H

class Imu
{
private:
#if defined IMU_MPU9250
    MPU9250 mpu;
#elif defined IMU_MPU6050
    MPU6050 mpu;
#elif defined IMU_LSM9DS1
#define mpu IMU
#endif

public:
    void calibrate();
    void init();
    void getImu();
};
#endif