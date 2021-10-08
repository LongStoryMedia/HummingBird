#ifndef MPU_LOCAL_H
#define MPU_LOCAL_H

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

//Uncomment only one full scale accelerometer range (G's)
// #define ACCEL_2G //default
//#define ACCEL_4G
//#define ACCEL_8G
#define ACCEL_16G

#if defined IMU_MPU6050
#define GYRO_FS_SEL_250 MPU6050_GYRO_FS_250
#define GYRO_FS_SEL_500 MPU6050_GYRO_FS_500
#define GYRO_FS_SEL_1000 MPU6050_GYRO_FS_1000
#define GYRO_FS_SEL_2000 MPU6050_GYRO_FS_2000
#define ACCEL_FS_SEL_2 MPU6050_ACCEL_FS_2
#define ACCEL_FS_SEL_4 MPU6050_ACCEL_FS_4
#define ACCEL_FS_SEL_8 MPU6050_ACCEL_FS_8
#define ACCEL_FS_SEL_16 MPU6050_ACCEL_FS_16
#elif defined IMU_MPU9250
#define GYRO_FS_SEL_250 mpu9250.GYRO_RANGE_250DPS
#define GYRO_FS_SEL_500 mpu9250.GYRO_RANGE_500DPS
#define GYRO_FS_SEL_1000 mpu9250.GYRO_RANGE_1000DPS
#define GYRO_FS_SEL_2000 mpu9250.GYRO_RANGE_2000DPS
#define ACCEL_FS_SEL_2 mpu9250.ACCEL_RANGE_2G
#define ACCEL_FS_SEL_4 mpu9250.ACCEL_RANGE_4G
#define ACCEL_FS_SEL_8 mpu9250.ACCEL_RANGE_8G
#define ACCEL_FS_SEL_16 mpu9250.ACCEL_RANGE_16G
#endif

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

typedef struct
{
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
#if IMU_MPU9250
    MPU9250 mpu;
#else
    MPU6050 mpu;
#endif
#endif
    void getError();
    void IMUinit();
    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);
    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);
    void calibrateAttitude();

    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    uint32_t mpu_pt, mpu_t, mpu_dt;

    float AccX, AccY, AccZ;
    float AccX_prev, AccY_prev, AccZ_prev;
    float GyroX, GyroY, GyroZ;
    float GyroX_prev, GyroY_prev, GyroZ_prev;
    float MagX, MagY, MagZ;
    float MagX_prev, MagY_prev, MagZ_prev;
    float roll_IMU, pitch_IMU, yaw_IMU;
    float roll_IMU_prev, pitch_IMU_prev;
    float AccErrorX, AccErrorY, AccErrorZ, GyroErrorX, GyroErrorY, GyroErrorZ;
    float q0 = 1.0f; //initialize quaternion for madgwick filter
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    //Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
    float B_madgwick = 0.04; //Madgwick filter parameter
    float B_accel = 0.14;    //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    float B_gyro = 0.1;      //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    float B_mag = 1.0;       //Magnetometer LP filter parameter

    //Magnetometer calibration parameters - if using MPU9250, uncomment calibrateMagnetometer() in void setup() to get these values, else just ignore these
    float MagErrorX = 0.0;
    float MagErrorY = 0.0;
    float MagErrorZ = 0.0;
    float MagScaleX = 1.0;
    float MagScaleY = 1.0;
    float MagScaleZ = 1.0;

    Orientation orientation;

public:
    float rawYpr[3];

    static volatile bool mpuInterrupt; // indicates whether MPU interrupt pin has gone high
    static void dmpDataReady()
    {
        mpuInterrupt = true;
    }

    void calibrate();
    Orientation getOrientation(uint32_t dt);
};
#endif