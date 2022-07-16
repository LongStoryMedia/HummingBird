// Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS // default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS

// Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G // default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G

// #define USE_PWM

#define I_TH 100
#define KL 30.0f
#define I_LIMIT 20.0f        // Integrator saturation level, mostly for safety (default 25.0)
#define MAX_ROLL 30.0f       // Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_PITCH 30.0f      // Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_YAW 160.0f       // Max yaw rate in deg/sec
#define KP_ROLL_ANGLE 0.1f   // Roll P-gain - angle mode
#define KI_ROLL_ANGLE 0.18f  // Roll I-gain - angle mode
#define KD_ROLL_ANGLE 0.0f   // Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_ROLL 0.9f     // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_PITCH_ANGLE 0.15f // Pitch P-gain - angle mode
#define KI_PITCH_ANGLE 0.18f // Pitch I-gain - angle mode
#define KD_PITCH_ANGLE 0.0f  // Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_PITCH 0.9f    // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_ROLL_RATE 0.05f   // Roll P-gain - rate mode
#define KI_ROLL_RATE 0.09f   // Roll I-gain - rate mode
#define KD_ROLL_RATE 0.0f    // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_PITCH_RATE 0.05f  // Pitch P-gain - rate mode
#define KI_PITCH_RATE 0.09f  // Pitch I-gain - rate mode
#define KD_PITCH_RATE 0.0f   // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_YAW 0.1f          // Yaw P-gain
#define KI_YAW 0.15f         // Yaw I-gain
#define KD_YAW 0.000001f     // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
#define KP_ALT 0.175f
#define KI_ALT 0.15f
#define ALT_CONSTRAINT_ABS 0.01f
#define ALT_CONSTRAINT_INC 0.0000001f

// #define OFFSET_X_ACCEL -1130
// #define OFFSET_Y_ACCEL -3047
// #define OFFSET_Z_ACCEL 1400
// #define OFFSET_X_GYRO -27
// #define OFFSET_Y_GYRO 47
// #define OFFSET_Z_GYRO 34

// 743 thrust
// all 0 m                            m1: 174, m2: 194, m3: 250, m4: 245
// just offset_x_accel @ -1130 -      m1: 224, m2: 217, m3: 210, m4: 217
//// with offset_z_accel @ 1400 -     m1: 250, m2: 249, m3: 182, m4: 186
//// with offset_z_accel @ 100 -      m1: 211, m2: 212, m3: 223, m4: 222
//// with offset_z_accel @ 300 -      m1: 217, m2: 218, m3: 217, m4: 216 ***!!***
////// with offset_y_accel @ -100 -   m1: 243, m2: 186, m3: 191, m4: 249
////// with offset_y_accel @ 100 -    m1: 187, m2: 243, m3: 250, m4: 188
// ***!!*** with time gyro defaults - m1: 228, m2: 217, m3: 207, m4: 217
// gx 27                              m1: 214, m2: 231, m3: 221, m4: 204
// gx 0, gy 0                         m1: 203, m2: 214, m3: 230, m4: 220
// gx 0, gy 20                        m1: 217, m2: 228, m3: 217, m4: 207
// gx -5, gy 10                       m1: 215, m2: 205, m3: 217, m4: 228
// gx -2, gy 5                        m1: 206, m2: 216, m3: 228, m4: 218
// gx -3, gy 6                        m1: 221, m2: 209, m3: 214, m4: 226

#define OFFSET_X_ACCEL -1991
#define OFFSET_Y_ACCEL -2929
#define OFFSET_Z_ACCEL 1722
#define OFFSET_X_GYRO 98
#define OFFSET_Y_GYRO 105
#define OFFSET_Z_GYRO -14
#define OFFSET(type, axis) OFFSET_##axis_##type

// hardware configuration
#define M1_PIN 2
#define M2_PIN 4
#define M3_PIN 5
#define M4_PIN 3

#define CE_NRF24 10
#define CSN_NRF24 9

#define PID_MODE 1        // simple angle = 0, cascading angle = 1, simple rate = 2
#define IMU_ORIENTATION 1 // up/forward = 0, up/backward = 1, down/forward = 2, down/backward = 3
#define PROP_CONFIG 1

#define USE_BMP390
#define USE_MULTISHOT

#define ALT_WIRE Wire
#define IMU_WIRE Wire

// #define USS_DN_PIN 22
// #define USS_TRIG_PIN 23

// #define CALIBRATION_MODE
// averaging 10000 readings each time
// XAccel                         YAccel                          ZAccel                          XGyro                   YGyro                 ZGyro
// [-1117,-1116] --> [-3,14]      [-3027,-3025] --> [0,19]        [1389,1390] --> [16376,16391]   [-19,-19] --> [0,1]     [48,49] --> [-3,1]    [32,33] --> [0,2]
// [-1117,-1116] --> [-5,14]      [-3027,-3026] --> [0,20]        [1389,1390] --> [16375,16391]   [-19,-19] --> [0,1]     [48,49] --> [-3,1]    [32,33] --> [0,2]
// [-1117,-1116] --> [-3,14]      [-3027,-3026] --> [0,20]        [1389,1390] --> [16376,16391]   [-19,-19] --> [0,1]     [48,49] --> [-2,1]    [32,33] --> [0,2]
// -------------- done --------------

// averaging 10000 readings each time
// XAccel                         YAccel                          ZAccel                          XGyro                   YGyro                   ZGyro
// [-1147,-1146] --> [-5,10]      [-3052,-3050] --> [-9,9]        [1389,1390] --> [16371,16387]   [-22,-21] --> [-4,1]    [48,49] --> [0,4]       [32,33] --> [-3,2]
// [-1147,-1146] --> [-3,10]      [-3051,-3050] --> [-10,9]       [1389,1390] --> [16370,16387]   [-22,-21] --> [-3,1]    [48,49] --> [0,4]       [32,33] --> [-2,2]
// [-1147,-1146] --> [-1,10]      [-3051,-3050] --> [-11,9]       [1389,1390] --> [16369,16387]   [-22,-21] --> [-3,1]    [48,49] --> [0,4]       [32,33] --> [-1,2]
// -------------- done --------------

// #define USE_STATIC_OFFSETS