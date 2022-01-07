#include "config.h"

void Imu::init()
{
#if defined USE_MPU6050_I2C
    Wire.begin();
    Wire.setClock(1000000); // Note this is 2.5 times the spec sheet 400 kHz max...

    mpu.initialize();

    if (mpu.testConnection() == false)
    {
        Serial.println("MPU6050 initialization unsuccessful");
        Serial.println("Check MPU6050 wiring or try cycling power");
        while (1)
        {
        }
    }

    // From the reset state all registers should be 0x00, so we should be at
    // max sample rate with digital low pass filter(s) off.  All we need to
    // do is set the desired fullscale ranges
    mpu.setFullScaleGyroRange(GYRO_SCALE);
    mpu.setFullScaleAccelRange(ACCEL_SCALE);

#elif defined USE_MPU9250_SPI
    int status = mpu.begin();

    if (status < 0)
    {
        Serial.println("MPU9250 initialization unsuccessful");
        Serial.println("Check MPU9250 wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1)
        {
        }
    }

    // From the reset state all registers should be 0x00, so we should be at
    // max sample rate with digital low pass filter(s) off.  All we need to
    // do is set the desired fullscale ranges
    mpu.setGyroRange(GYRO_SCALE);
    mpu.setAccelRange(ACCEL_SCALE);
    mpu.setMagCalX(MagErrorX, MagScaleX);
    mpu.setMagCalY(MagErrorY, MagScaleY);
    mpu.setMagCalZ(MagErrorZ, MagScaleZ);
    mpu.setSrd(0); // sets gyro and accel read to 1khz, magnetometer read to 100hz
#endif
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;

    // Read IMU values 12000 times
    int c = 0;
    while (c < 12000)
    {
#if defined USE_MPU6050_I2C
        mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
        mpu.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
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
    // Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++)
    {
        timer.update();
        getImu();
        Madgwick(ag.gyro.roll, -ag.gyro.pitch, -ag.gyro.yaw, -ag.accel.roll, ag.accel.pitch, ag.accel.yaw, ag.mag.pitch, -ag.mag.roll, ag.mag.yaw);
        loopRate(2000); // do not exceed 2000Hz
    }
#if defined USE_MPU9250_SPI
    float success;
    Serial.println("Beginning magnetometer calibration in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Rotate the IMU about all axes until complete.");
    Serial.println(" ");
    success = mpu9250.calibrateMag();
    if (success)
    {
        Serial.println("Calibration Successful!");
        Serial.println("Please comment out the calibrateMagnetometer() function and copy these values into the code:");
        Serial.print("float MagErrorX = ");
        Serial.print(mpu9250.getMagBiasX_uT());
        Serial.println(";");
        Serial.print("float MagErrorY = ");
        Serial.print(mpu9250.getMagBiasY_uT());
        Serial.println(";");
        Serial.print("float MagErrorZ = ");
        Serial.print(mpu9250.getMagBiasZ_uT());
        Serial.println(";");
        Serial.print("float MagScaleX = ");
        Serial.print(mpu9250.getMagScaleFactorX());
        Serial.println(";");
        Serial.print("float MagScaleY = ");
        Serial.print(mpu9250.getMagScaleFactorY());
        Serial.println(";");
        Serial.print("float MagScaleZ = ");
        Serial.print(mpu9250.getMagScaleFactorZ());
        Serial.println(";");
        Serial.println(" ");
        Serial.println("If you are having trouble with your attitude estimate at a new flying location, repeat this process as needed.");
    }
    else
    {
        Serial.println("Calibration Unsuccessful. Please reset the board and try again.");
    }

    while (1)
        ; // halt code so it won't enter main loop until this function commented out
#endif
}

void Imu::getImu()
{
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;

#if defined USE_MPU6050_I2C
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);
#elif defined USE_MPU9250_SPI
    mpu.getMotion9(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ, &MgX, &MgY, &MgZ);
#endif

    // Accelerometer
    ag.accel.roll = AcX / ACCEL_SCALE_FACTOR; // G's ex 700/16382=0.0427
    ag.accel.pitch = AcY / ACCEL_SCALE_FACTOR;
    ag.accel.yaw = AcZ / ACCEL_SCALE_FACTOR;
    // Correct the outputs with the calculated error values
    ag.accel -= agError.accel;
    // LP filter accelerometer data
    ag.accel.roll = (1.0 - filter.accel) * agPrev.accel.roll + filter.accel * ag.accel.roll; // 0.86 * 0 * 0.14 * -0.007 = 0.0000058996
    ag.accel.pitch = (1.0 - filter.accel) * agPrev.accel.pitch + filter.accel * ag.accel.pitch;
    ag.accel.yaw = (1.0 - filter.accel) * agPrev.accel.yaw + filter.accel * ag.accel.yaw;
    agPrev.accel = ag.accel;

    // Gyro
    ag.gyro.roll = GyX / GYRO_SCALE_FACTOR; // deg/sec
    ag.gyro.pitch = GyY / GYRO_SCALE_FACTOR;
    ag.gyro.yaw = GyZ / GYRO_SCALE_FACTOR;
    // Correct the outputs with the calculated error values
    ag.gyro -= agError.gyro;
    // LP filter gyro data
    ag.gyro.roll = (1.0 - filter.gyro) * agPrev.gyro.roll + filter.gyro * ag.gyro.roll;
    ag.gyro.pitch = (1.0 - filter.gyro) * agPrev.gyro.pitch + filter.gyro * ag.gyro.pitch;
    ag.gyro.yaw = (1.0 - filter.gyro) * agPrev.gyro.yaw + filter.gyro * ag.gyro.yaw;
    agPrev.gyro = ag.gyro;

    // Magnetometer
    ag.mag.roll = MgX / 6.0; // uT
    ag.mag.pitch = MgY / 6.0;
    ag.mag.yaw = MgZ / 6.0;
    // Correct the outputs with the calculated error values
    ag.mag.roll = (ag.mag.roll - agError.mag.roll) * 1.0;
    ag.mag.pitch = (ag.mag.pitch - agError.mag.pitch) * 1.0;
    ag.mag.yaw = (ag.mag.yaw - agError.mag.yaw) * 1.0;
    // LP filter magnetometer data
    ag.mag.roll = (1.0 - filter.mag) * agPrev.mag.roll + filter.mag * ag.mag.roll;
    ag.mag.pitch = (1.0 - filter.mag) * agPrev.mag.pitch + filter.mag * ag.mag.pitch;
    ag.mag.yaw = (1.0 - filter.mag) * agPrev.mag.yaw + filter.mag * ag.mag.yaw;
    agPrev.mag = ag.mag;
}