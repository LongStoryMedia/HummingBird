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
#define USE_MPL3115A2
#define X1_CONFIG
#define I_TH 500

#define I_LIMIT 25.0f         // Integrator saturation level, mostly for safety (default 25.0)
#define MAX_ROLL 30.0f        // Max roll angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_PITCH 30.0f       // Max pitch angle in degrees for angle mode (maximum 60 degrees), deg/sec for rate mode
#define MAX_YAW 160.0f        // Max yaw rate in deg/sec
#define KP_ROLL_ANGLE 0.325f  // Roll P-gain - angle mode
#define KI_ROLL_ANGLE 0.025f  // Roll I-gain - angle mode
#define KD_ROLL_ANGLE 0.04f   // Roll D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_ROLL 0.9f      // Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_PITCH_ANGLE 0.55f  // Pitch P-gain - angle mode
#define KI_PITCH_ANGLE 0.02f  // Pitch I-gain - angle mode
#define KD_PITCH_ANGLE 0.04f  // Pitch D-gain - angle mode (if using controlANGLE2(), set to 0.0)
#define B_LOOP_PITCH 0.9f     // Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
#define KP_ROLL_RATE 0.15f    // Roll P-gain - rate mode
#define KI_ROLL_RATE 0.02f    // Roll I-gain - rate mode
#define KD_ROLL_RATE 0.0002f  // Roll D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_PITCH_RATE 0.15f   // Pitch P-gain - rate mode
#define KI_PITCH_RATE 0.2f    // Pitch I-gain - rate mode
#define KD_PITCH_RATE 0.0002f // Pitch D-gain - rate mode (be careful when increasing too high, motors will begin to overheat!)
#define KP_YAW 0.9f           // Yaw P-gain
#define KI_YAW 0.5f           // Yaw I-gain
#define KD_YAW 0.000025f      // Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
#define KP_ALT 0.25f
#define KI_ALT 0.4f

#define PID_MODE 0 // simple angle = 0, cascading angle = 1, simple rate = 2

#define M1_PIN 2
#define M2_PIN 3
#define M3_PIN 4
#define M4_PIN 5