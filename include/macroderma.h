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

#define I_TH 200
#define KL 30.0f
#define I_LIMIT 20.0f        // Integrator saturation level, mostly for safety (default 25.0)
#define MAX_ROLL 30.0f       // Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_PITCH 30.0f      // Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_YAW 160.0f       // Max yaw rate in deg/sec
#define KP_ROLL_ANGLE 0.095f // Roll P-gain - angle mode
#define KI_ROLL_ANGLE 0.25f  // Roll I-gain - angle mode
#define KD_ROLL_ANGLE 0.0f   // Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_ROLL 0.9f     // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_PITCH_ANGLE 0.11f // Pitch P-gain - angle mode
#define KI_PITCH_ANGLE 0.25f // Pitch I-gain - angle mode
#define KD_PITCH_ANGLE 0.0f  // Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_PITCH 0.9f    // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_ROLL_RATE 0.1f    // Roll P-gain - rate mode
#define KI_ROLL_RATE 0.2f    // Roll I-gain - rate mode
#define KD_ROLL_RATE 0.0f    // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_PITCH_RATE 0.095f // Pitch P-gain - rate mode
#define KI_PITCH_RATE 0.15f  // Pitch I-gain - rate mode
#define KD_PITCH_RATE 0.0f   // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_YAW 0.065f        // Yaw P-gain
#define KI_YAW 0.2f          // Yaw I-gain
#define KD_YAW 0.00001f      // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
#define KP_ALT 0.175f
#define KI_ALT 0.15f
#define ALT_CONSTRAINT_ABS 0.01f
#define ALT_CONSTRAINT_INC 0.0000001f

#define OFFSET_X_ACCEL -1130
#define OFFSET_Y_ACCEL -3047
#define OFFSET_Z_ACCEL 1400
#define OFFSET_X_GYRO -27
#define OFFSET_Y_GYRO 47
#define OFFSET_Z_GYRO 34
#define OFFSET(type, axis) OFFSET_##axis_##type

// hardware configuration
#define M1_PIN 3
#define M2_PIN 5
#define M3_PIN 4
#define M4_PIN 2

#define CE_NRF24 10
#define CSN_NRF24 9

#define PID_MODE 1        // simple angle = 0, cascading angle = 1, simple rate = 2
#define IMU_ORIENTATION 1 // up/forward = 0, up/backward = 1, down/forward = 2, down/backward = 3
#define PROP_CONFIG 1

#define USE_BMP390
// #define USE_MULTISHOT

#define ALT_WIRE Wire
#define IMU_WIRE Wire

// #define CALIBRATION_MODE
