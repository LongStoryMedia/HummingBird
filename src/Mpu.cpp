#include "config.h"

#if ACCGYROEXTERN
volatile bool Mpu::mpuInterrupt = false;
#endif

void Mpu::calibrate()
{
#if ACCGYROEXTERN

    Wire.begin();
    Wire.setClock(400000L);
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
    // mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
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
};

void Mpu::setSpace()
{
#if ACCGYROEXTERN
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

        // if (IMU.accelerationAvailable())
        // {
        //     IMU.readAcceleration(rawYpr[roll], rawYpr[pitch], rawYpr[yaw]);
        // }
        ypr[yaw] = rawYpr[yaw] * 180 / PI;
        ypr[pitch] = rawYpr[pitch] * 180 / PI;
        ypr[roll] = rawYpr[roll] * 180 / PI;
    }
#endif
};
