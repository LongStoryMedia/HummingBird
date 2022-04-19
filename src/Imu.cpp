#include "config.h"

#if defined(CALIBRATION_MODE)
#include "CalculateImuOffsets.h"
#endif

void Imu::init(TwoWire *wire)
{
    mpu.initialize(wire);

    if (mpu.testConnection() == false)
    {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        delay(5000);
        init(wire);
    }

    // From the reset state all registers should be 0x00, so we should be at
    // max sample rate with digital low pass filter(s) off.  All we need to
    // do is set the desired fullscale ranges
    mpu.setFullScaleGyroRange(GYRO_SCALE);
    mpu.setFullScaleAccelRange(ACCEL_SCALE);

#if defined IMU_LSM9DS1
    mpu.begin();
    mpu.setAccelFS(3);  // ± 2g
    mpu.setGyroFS(4);   // ± 245 °/s
    mpu.setMagnetFS(0); // ± 400 µT
    float AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
#else
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
#endif
    // Read IMU values 12000 times
    int c = 0;
    while (c < 12000)
    {

#if defined IMU_LSM9DS1
        mpu.readAccel(AcX, AcY, AcZ); //  Accelerometer returns G Force (ms-2)
        mpu.readGyro(GyX, GyY, GyZ);  //  Gyro rates are Degrees Per Second (DPS)
        mpu.readMagnet(MgX, MgY, MgZ);
#else
        mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#endif
        ag.accel.roll = AcX / ACCEL_SCALE_FACTOR;
        ag.accel.pitch = AcY / ACCEL_SCALE_FACTOR;
        ag.accel.yaw = AcZ / ACCEL_SCALE_FACTOR;
        ag.gyro.roll = GyX / GYRO_SCALE_FACTOR;
        ag.gyro.pitch = GyY / GYRO_SCALE_FACTOR;
        ag.gyro.yaw = GyZ / GYRO_SCALE_FACTOR;

        // Sum all readings
        agError += ag;
        c++;
    }
    // Divide the sum by 12000 to get the error value
    agError.accel.roll /= c;
    agError.accel.pitch /= c;
    agError.accel.yaw = agError.accel.yaw / c - 1.0;
    agError.gyro /= c;
}

void Imu::calibrate()
{
    // mpu.CalibrateAccel();
    // mpu.CalibrateGyro();

#if defined(CALIBRATION_MODE)
    for (int i = 0; i <= 50000; i++)
    {
        timer.update();
        getImu();
        timer.regulate();
    }
    CalculateOffsets(mpu);
    while (1)
        ;
#endif
#if defined(USE_STATIC_OFFSETS)
    mpu.setXAccelOffset(OFFSET_X_ACCEL);
    mpu.setYAccelOffset(OFFSET_Y_ACCEL);
    mpu.setZAccelOffset(OFFSET_Z_ACCEL);
    mpu.setXGyroOffset(OFFSET_X_GYRO);
    mpu.setYGyroOffset(OFFSET_Y_GYRO);
    mpu.setZGyroOffset(OFFSET_Z_GYRO);
#endif
    // Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++)
    {
        timer.update();
        getImu();
        timer.regulate();
    }
}

AccelGyro Imu::getImu()
{
#if defined IMU_LSM9DS1
    float AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
    mpu.readAccel(AcX, AcY, AcZ); //  Accelerometer returns G Force (ms-2)
    mpu.readGyro(GyX, GyY, GyZ);  //  Gyro rates are Degrees Per Second (DPS)
    mpu.readMagnet(MgX, MgY, MgZ);
#else
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#endif
    // Accelerometer
    ag.accel.roll = AcX / ACCEL_SCALE_FACTOR; // G's ex 700/16382=0.0427
    ag.accel.pitch = AcY / ACCEL_SCALE_FACTOR;
    ag.accel.yaw = AcZ / ACCEL_SCALE_FACTOR;
    // Gyro
    ag.gyro.roll = GyX / GYRO_SCALE_FACTOR; // deg/sec
    ag.gyro.pitch = GyY / GYRO_SCALE_FACTOR;
    ag.gyro.yaw = GyZ / GYRO_SCALE_FACTOR;
#if !defined(USE_STATIC_OFFSETS)
    ag -= agError;
#endif
    // LP filter accelerometer/gyro data
    ag = agPrev * (1.0f - filter.accel) + ag * filter.accel;
    agPrev = ag;

    return filterAg();
}

AccelGyro Imu::filterAg()
{
    // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
    /*
     * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
     * available (for example when using the recommended MPU6050 IMU for the default setup).
     */
    float recipNorm;

    // Convert gyroscope degrees/sec to radians/sec
    float gx = ag.gyro.roll * 0.0174533f;
    float gy = -ag.gyro.pitch * 0.0174533f;
    float gz = -ag.gyro.yaw * 0.0174533f;
    float ax = -ag.accel.roll;
    float ay = ag.accel.pitch;
    float az = ag.accel.yaw;

    // Rate of change of quaternion from gyroscope
    float qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
    float qDot2 = 0.5f * (q.q0 * gx + q.q2 * gz - q.q3 * gy);
    float qDot3 = 0.5f * (q.q0 * gy - q.q1 * gz + q.q3 * gx);
    float qDot4 = 0.5f * (q.q0 * gz + q.q1 * gy - q.q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        float _2q0 = 2.0f * q.q0;
        float _2q1 = 2.0f * q.q1;
        float _2q2 = 2.0f * q.q2;
        float _2q3 = 2.0f * q.q3;
        float _4q0 = 4.0f * q.q0;
        float _4q1 = 4.0f * q.q1;
        float _4q2 = 4.0f * q.q2;
        float _8q1 = 8.0f * q.q1;
        float _8q2 = 8.0f * q.q2;
        float q0q0 = q.q0 * q.q0;
        float q1q1 = q.q1 * q.q1;
        float q2q2 = q.q2 * q.q2;
        float q3q3 = q.q3 * q.q3;

        // Gradient decent algorithm corrective step
        float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        float s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        float s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;

        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= filter.madgwick * s0;
        qDot2 -= filter.madgwick * s1;
        qDot3 -= filter.madgwick * s2;
        qDot4 -= filter.madgwick * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q.q0 += qDot1 * timer.delta;
    q.q1 += qDot2 * timer.delta;
    q.q2 += qDot3 * timer.delta;
    q.q3 += qDot4 * timer.delta;

    // Normalise quaternion
    recipNorm = invSqrt(q.q0 * q.q0 + q.q1 * q.q1 + q.q2 * q.q2 + q.q3 * q.q3);
    q.q0 *= recipNorm;
    q.q1 *= recipNorm;
    q.q2 *= recipNorm;
    q.q3 *= recipNorm;

    AccelGyro filteredAg;
    filteredAg.gyro = ag.gyro;
    // compute angles
    filteredAg.accel.roll = atan2(q.q0 * q.q1 + q.q2 * q.q3, 0.5f - q.q1 * q.q1 - q.q2 * q.q2) * 57.29577951; // degrees
    filteredAg.accel.pitch = -asin(-2.0f * (q.q1 * q.q3 - q.q0 * q.q2)) * 57.29577951;                        // degrees
    filteredAg.accel.yaw = -atan2(q.q1 * q.q2 + q.q0 * q.q3, 0.5f - q.q2 * q.q2 - q.q3 * q.q3) * 57.29577951; // degrees

    // Serial.print("accRoll:");
    // Serial.print(filteredAg.accel.roll);
    // Serial.print("\taccPitch:");
    // Serial.print(filteredAg.accel.pitch);
    // Serial.print("\taccYaw:");
    // Serial.print(filteredAg.accel.yaw);
    // Serial.print("\tgyroRoll:");
    // Serial.print(filteredAg.gyro.roll);
    // Serial.print("\tgyroPitch:");
    // Serial.print(filteredAg.gyro.pitch);
    // Serial.print("\tgyroYaw:");
    // Serial.println(filteredAg.gyro.yaw);
    return filteredAg;
}