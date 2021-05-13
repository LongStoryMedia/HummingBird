#include "config.h"

#if ACCGYROEXTERN
volatile bool Mpu::mpuInterrupt = false;
#endif

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G
#if defined GYRO_250DPS
#define GYRO_SCALE GYRO_FS_SEL_250
#define GYRO_SCALE_FACTOR 131.0
#elif defined GYRO_500DPS
#define GYRO_SCALE GYRO_FS_SEL_500
#define GYRO_SCALE_FACTOR 65.5
#elif defined GYRO_1000DPS
#define GYRO_SCALE GYRO_FS_SEL_1000
#define GYRO_SCALE_FACTOR 32.8
#elif defined GYRO_2000DPS
#define GYRO_SCALE GYRO_FS_SEL_2000
#define GYRO_SCALE_FACTOR 16.4
#endif

#if defined ACCEL_2G
#define ACCEL_SCALE ACCEL_FS_SEL_2
#define ACCEL_SCALE_FACTOR 16384.0
#elif defined ACCEL_4G
#define ACCEL_SCALE ACCEL_FS_SEL_4
#define ACCEL_SCALE_FACTOR 8192.0
#elif defined ACCEL_8G
#define ACCEL_SCALE ACCEL_FS_SEL_8
#define ACCEL_SCALE_FACTOR 4096.0
#elif defined ACCEL_16G
#define ACCEL_SCALE ACCEL_FS_SEL_16
#define ACCEL_SCALE_FACTOR 2048.0
#endif

void Mpu::calibrate()
{
#if defined ACCGYROEXTERN
    Wire.begin();
    delay(2000);

    Wire.setClock(400000L);
#if defined IMU_MPU9250
    while (!mpu.setup(0x68))
    { // change to your own address
        Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
        delay(5000);
    }
    mpu.verbose(false);
    mpu.calibrateAccelGyro();
    // AccErrorX = mpu.getAccBiasX();
    // AccErrorY = mpu.getAccBiasY();
    // AccErrorZ = mpu.getAccBiasZ();
    // GyroErrorX = mpu.getGyroBiasX();
    // GyroErrorY = mpu.getGyroBiasY();
    // GyroErrorZ = mpu.getGyroBiasZ();

#else
    mpu.initialize();
// load and configure the DMP
#if DEBUG
    Serial.println(F("Initializing DMP..."));
#endif
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // mpu.setXGyroOffset(220);
    // mpu.setYGyroOffset(76);
    // mpu.setZGyroOffset(-126);
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip   // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
// turn on the DMP, now that it's ready
#if DEBUG
        Serial.println(F("Enabling DMP..."));
#endif
        mpu.setDMPEnabled(true);

// enable Arduino interrupt detection
#if DEBUG
        Serial.println(F("Enabling interrupt detection..."));
#endif
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

// set our DMP Ready flag so the main loop() function knows it's okay to use it
#if DEBUG
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
#endif
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        getError();
    }
    else
    {
// ERROR!
// 1 = initial memory load failed
// 2 = DMP configuration updates failed
// (if it's going to break, usually the code will be 1)
#if DEBUG
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
#endif
    }
#endif
#else
    IMU.begin();
#endif
};

Orientation Mpu::getOrientation()
{
#if ACCGYROEXTERN
#if IMU_MPU9250
    if (mpu.update())
    {
        orientation.yaw = mpu.getYaw();
        orientation.pitch = mpu.getPitch();
        orientation.roll = mpu.getRoll();
        orientation.ax = mpu.getAccX();
        orientation.ay = mpu.getAccY();
        orientation.az = mpu.getAccZ();
        orientation.gx = mpu.getGyroX();
        orientation.gy = mpu.getGyroY();
        orientation.gz = mpu.getGyroZ();
    }
    // correct();
    return orientation;
#else
    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
        return;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
#if DEBUG
        Serial.println(F("FIFO overflow!"));
#endif
        return;
    }
    if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(rawYpr, &q, &gravity);

        ypr[yaw] = rawYpr[yaw] * 180 / PI;
        ypr[pitch] = rawYpr[pitch] * 180 / PI;
        ypr[roll] = rawYpr[roll] * 180 / PI;
        correct();
        ax -= AccErrorX;
        ay -= AccErrorY;
        az -= AccErrorZ;
        gx -= GyroErrorX;
        gy -= GyroErrorY;
        gz -= GyroErrorZ;
    }
#endif
#else
    if (IMU.accelerationAvailable())
    {
        IMU.readAcceleration(rawYpr[0], rawYpr[1], rawYpr[2]);
    }
    orientation.yaw = rawYpr[0] * 180 / PI;
    orientation.pitch = rawYpr[1] * 180 / PI;
    orientation.roll = rawYpr[2] * 180 / PI;
    return orientation;
#endif
};

void Mpu::getError()
{
#if defined IMU_MPU6050

    //DESCRIPTION: Computes IMU accelerometer and gyro error on startup. Note: vehicle should be powered up on flat surface
    /*
   * Don't worry too much about what this is doing. The error values it computes are applied to the raw gyro and 
   * accelerometer values AccX, AccY, AccZ, GyroX, GyroY, GyroZ in getIMUdata(). This eliminates drift in the
   * measurement. 
   */

    //Read IMU values 12000 times
    int c = 0;
    while (c < 12000)
    {
        correct();
        //     //Sum all readings
        AccErrorX += ax;
        AccErrorY += ay;
        AccErrorZ += az;
        GyroErrorX += gx;
        GyroErrorY += gy;
        GyroErrorZ += gz;
        c++;
    }
// //Divide the sum by 12000 to get the error value
// AccErrorX = AccErrorX / c;
// AccErrorY = AccErrorY / c;
// AccErrorZ = AccErrorZ / c - 1.0;
// GyroErrorX = GyroErrorX / c;
// GyroErrorY = GyroErrorY / c;
// GyroErrorZ = GyroErrorZ / c;
// }
#endif
};

void Mpu::correct()
{
#if defined IMU_MPU6050
    int16_t AcX, AcY, AcZ, GyX, GyY, GyZ, MgX, MgY, MgZ;
    mpu.getMotion6(&AcX, &AcY, &AcZ, &GyX, &GyY, &GyZ);

#endif
    orientation.ax /= ACCEL_SCALE_FACTOR;
    orientation.ay /= ACCEL_SCALE_FACTOR;
    orientation.az /= ACCEL_SCALE_FACTOR;
    orientation.gx /= GYRO_SCALE_FACTOR;
    orientation.gy /= GYRO_SCALE_FACTOR;
    orientation.gz /= GYRO_SCALE_FACTOR;
}