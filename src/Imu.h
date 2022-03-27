#ifndef IMU_LOCAL_H
#define IMU_LOCAL_H

class Imu
{
private:
    MPU6050 mpu;

public:
    void calibrate();
    void init(TwoWire *wire);
    void getImu();

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