#include "config.h"

//========================================================================================================================//
//                                                 GLOBALS                                                                //
//========================================================================================================================//

// uint32_t print_counter, serial_counter;
// uint32_t blink_counter, blink_delay;

unsigned long print_counter;
bool blinkAlternate;
Timer timer{0.0, 0, 0, 2000};
Filter filter{0.04, 0.14, 0.1, 1.0};
Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
AccelGyro ag;
AccelGyro agPrev;
AccelGyro agError;
AccelGyro agImu;
AccelGyro agImuPrev;
State packet;
State prevPacket;
PropConfig propConfig;
Rx rx;
Esc esc;
Pid pid;
Imu imu;
#if defined(USE_MPL3115A2)
Alt alt;
#endif
#if defined(USE_PROXIMITY_DETECTION)
Proximity proximity;
#endif
//========================================================================================================================//
//                                                 SETUP                                                                  //
//========================================================================================================================//

void setup()
{
  Serial.begin(115200); // usb serial
  // Initialize all pins
  pinMode(13, OUTPUT); // pin 13 LED blinker on board, do not modify

  // Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(13, HIGH);

  delay(10);
  pid.init();
  // Initialize radio communication
  rx.init();
#if defined(USE_MPL3115A2)
  // Initialize Baro communication
  alt.init();
  delay(10);
#endif

#if defined(USE_PROXIMITY_DETECTION)
  proximity.init();
  delay(10);
#endif

  // Initialize IMU communication
  imu.init();
  delay(10);
  esc.arm();
  delay(100);
  // Warm up the loop
  imu.calibrate(); // helps to warm up IMU and Madgwick filter before finally entering main loop
  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
}

//========================================================================================================================//
//                                                       MAIN LOOP                                                        //
//========================================================================================================================//

void loop()
{
  timer.update();
  loopBlink(); // indicate we are in main loop with short blink every 1.5 seconds
  // Get vehicle state
  imu.getImu();
  Madgwick(ag.gyro.roll, -ag.gyro.pitch, -ag.gyro.yaw, -ag.accel.roll, ag.accel.pitch, ag.accel.yaw, ag.mag.pitch, -ag.mag.roll, ag.mag.yaw);
  // updates agImu.accel.roll, agImu.accel.pitch, and agImu.accel.yaw (degrees)
  packet = rx.getPacket();

  pid.setDesiredState(); // convert raw commands to normalized values based on saturated control limits
  Commands commands = pid.control(agImu);

  if (packet.thrust < 10)
  {
#if defined(ESC_PROGRAM_MODE)
    commands = 125;
  }
  if (packet.thrust > 50)
  {
    commands = 250;
  }
#else
    commands = 130;
  }
#endif

#if defined(USE_MPL3115A2)
  alt.altCheck();
  // debug(alt.getAlt());
#endif

#if defined(USE_PROXIMITY_DETECTION)
  obstacles obs = proximity.scan();
#endif
  // if (timer.now - print_counter > 100000)
  // {
  // Serial.println(obs.hasObstacles());
  // Serial.print(packet.thrust);
  // Serial.print(" | ");
  // Serial.print(packet.yaw);
  // Serial.print(" | ");
  // Serial.print(packet.roll);
  // Serial.print(" | ");
  // Serial.print(packet.pitch);
  // Serial.print(" | ");
  // Serial.println(packet.lockAlt);
  // }
  esc.setSpeed(commands);
  // Regulate loop rate
  loopRate(); // do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
  // debug(timer.delta * 1000000);
}

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

void loopRate()
{
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (hzToUs(timer.loopRate) > (checker - timer.now))
  {
    checker = micros();
  }
}

float hzToUs(int speed)
{
  return 1.0 / speed * 1000000.0;
}

void loopBlink()
{
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
  unsigned long blink_counter;
  unsigned long blink_delay;
  if (timer.now - blink_counter > blink_delay)
  {
    blink_counter = micros();
    digitalWrite(13, blinkAlternate ? HIGH : LOW); // pin 13 is built in LED

    if (blinkAlternate == 1)
    {
      blinkAlternate = 0;
      blink_delay = 100000;
    }
    else if (blinkAlternate == 0)
    {
      blinkAlternate = 1;
      blink_delay = 2000000;
    }
  }
}

void setupBlink(int numBlinks, int upTime, int downTime)
{
  // DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++)
  {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}

//=========================================================================================//

// HELPER FUNCTIONS

float invSqrt(float x)
{
  // Fast inverse sqrt for madgwick filter
  /*
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  */
  // alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int *)&x >> 1);
  float tmp = *(float *)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

template <class T>
void debug(T data)
{
  if (timer.now - print_counter > 10000)
  {
    print_counter = micros();
    Serial.println(data);
  }
}