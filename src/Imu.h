#ifndef IMU_LOCAL_H
#define IMU_LOCAL_H

class Imu
{
private:
    MPU6050 mpu;
    AccelGyro filterAg();
    AccelGyro ag;
    AccelGyro agPrev;
    AccelGyro agError;
    Filter filter{0.04, 0.14, 0.1, 1.0};
    Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};

public:
    void
    calibrate();
    void init(TwoWire *wire);
    AccelGyro getImu();

#if defined(IMU_MPU9250)
    const MPU9250::GyroRange GYRO_RANGE_250DPS = mpu.GYRO_RANGE_250DPS;
    const MPU9250::GyroRange GYRO_RANGE_500DPS = mpu.GYRO_RANGE_500DPS;
    const MPU9250::GyroRange GYRO_RANGE_1000DPS = mpu.GYRO_RANGE_1000DPS;
    const MPU9250::GyroRange GYRO_RANGE_2000DPS = mpu.GYRO_RANGE_2000DPS;
    const MPU9250::AccelRange ACCEL_RANGE_2G = mpu.ACCEL_RANGE_2G;
    const MPU9250::AccelRange ACCEL_RANGE_4G = mpu.ACCEL_RANGE_4G;
    const MPU9250::AccelRange ACCEL_RANGE_8G = mpu.ACCEL_RANGE_8G;
    const MPU9250::AccelRange ACCEL_RANGE_16G = mpu.ACCEL_RANGE_16G;
#endif
};
#endif