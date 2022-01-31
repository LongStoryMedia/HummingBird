#include "config.h"

void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
    // DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
    /*
     * This function fuses the accelerometer gyro, and magnetometer readings ag.accel.roll, ag.accel.pitch, ag.accel.yaw, ag.gyro.roll, ag.gyro.pitch, ag.gyro.yaw, ag.mag.roll, ag.mag.pitch, and ag.mag.yaw for attitude estimation.
     * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
     * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower
     * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the agImu.accel.roll,
     * agImu.accel.pitch, and agImu.accel.yaw variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
     */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

// use 6DOF algorithm if MPU6050 is being used
#if defined IMU_MPU6050
    Madgwick6DOF(gx, gy, gz, ax, ay, az);
    return;
#endif

    // Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
    {
        Madgwick6DOF(gx, gy, gz, ax, ay, az);
        return;
    }

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
    qDot2 = 0.5f * (q.q0 * gx + q.q2 * gz - q.q3 * gy);
    qDot3 = 0.5f * (q.q0 * gy - q.q1 * gz + q.q3 * gx);
    qDot4 = 0.5f * (q.q0 * gz + q.q1 * gy - q.q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q.q0 * mx;
        _2q0my = 2.0f * q.q0 * my;
        _2q0mz = 2.0f * q.q0 * mz;
        _2q1mx = 2.0f * q.q1 * mx;
        _2q0 = 2.0f * q.q0;
        _2q1 = 2.0f * q.q1;
        _2q2 = 2.0f * q.q2;
        _2q3 = 2.0f * q.q3;
        _2q0q2 = 2.0f * q.q0 * q.q2;
        _2q2q3 = 2.0f * q.q2 * q.q3;
        q0q0 = q.q0 * q.q0;
        q0q1 = q.q0 * q.q1;
        q0q2 = q.q0 * q.q2;
        q0q3 = q.q0 * q.q3;
        q1q1 = q.q1 * q.q1;
        q1q2 = q.q1 * q.q2;
        q1q3 = q.q1 * q.q3;
        q2q2 = q.q2 * q.q2;
        q2q3 = q.q2 * q.q3;
        q3q3 = q.q3 * q.q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q.q3 + _2q0mz * q.q2 + mx * q1q1 + _2q1 * my * q.q2 + _2q1 * mz * q.q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q.q3 + my * q0q0 - _2q0mz * q.q1 + _2q1mx * q.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q.q3 - my * q3q3;
        _2bx = sqrtf(hx * hx + hy * hy);
        _2bz = -_2q0mx * q.q2 + _2q0my * q.q1 + mz * q0q0 + _2q1mx * q.q3 - mz * q1q1 + _2q2 * my * q.q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.q3 + _2bz * q.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.q2 + _2bz * q.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.q3 - _4bz * q.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q.q2 - _2bz * q.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q.q1 + _2bz * q.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q.q0 - _4bz * q.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q.q3 + _2bz * q.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q.q0 + _2bz * q.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
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

    agImu = ag;
    // compute angles - NWU
    agImu.accel.roll = atan2(q.q0 * q.q1 + q.q2 * q.q3, 0.5f - q.q1 * q.q1 - q.q2 * q.q2) * 57.29577951; // degrees
    agImu.accel.pitch = -asin(-2.0f * (q.q1 * q.q3 - q.q0 * q.q2)) * 57.29577951;                        // degrees
    agImu.accel.yaw = -atan2(q.q1 * q.q2 + q.q0 * q.q3, 0.5f - q.q2 * q.q2 - q.q3 * q.q3) * 57.29577951; // degrees
}

void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az)
{
    // DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
    /*
     * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
     * available (for example when using the recommended MPU6050 IMU for the default setup).
     */
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q.q1 * gx - q.q2 * gy - q.q3 * gz);
    qDot2 = 0.5f * (q.q0 * gx + q.q2 * gz - q.q3 * gy);
    qDot3 = 0.5f * (q.q0 * gy - q.q1 * gz + q.q3 * gx);
    qDot4 = 0.5f * (q.q0 * gz + q.q1 * gy - q.q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q.q0;
        _2q1 = 2.0f * q.q1;
        _2q2 = 2.0f * q.q2;
        _2q3 = 2.0f * q.q3;
        _4q0 = 4.0f * q.q0;
        _4q1 = 4.0f * q.q1;
        _4q2 = 4.0f * q.q2;
        _8q1 = 8.0f * q.q1;
        _8q2 = 8.0f * q.q2;
        q0q0 = q.q0 * q.q0;
        q1q1 = q.q1 * q.q1;
        q2q2 = q.q2 * q.q2;
        q3q3 = q.q3 * q.q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q.q3 - _2q1 * ax + 4.0f * q2q2 * q.q3 - _2q2 * ay;
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

    agImu = ag;
    // compute angles
    agImu.accel.roll = atan2(q.q0 * q.q1 + q.q2 * q.q3, 0.5f - q.q1 * q.q1 - q.q2 * q.q2) * 57.29577951; // degrees
    agImu.accel.pitch = -asin(-2.0f * (q.q1 * q.q3 - q.q0 * q.q2)) * 57.29577951;                        // degrees
    agImu.accel.yaw = -atan2(q.q1 * q.q2 + q.q0 * q.q3, 0.5f - q.q2 * q.q2 - q.q3 * q.q3) * 57.29577951; // degrees
}