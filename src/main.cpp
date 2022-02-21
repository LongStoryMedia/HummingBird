#include "config.h"

//========================================================================================================================//
//                                                 GLOBALS                                                                //
//========================================================================================================================//

uint32_t print_counter, serial_counter;
uint32_t blink_counter, blink_delay;
bool blinkAlternate;
Timer timer;
Filter filter{0.04, 0.14, 0.1, 1.0};
Quaternion q{1.0f, 0.0f, 0.0f, 0.0f};
AccelGyro ag;
AccelGyro agPrev;
AccelGyro agError;
AccelGyro agImu;
AccelGyro agImuPrev;
Rx rx;
Esc esc;
Pid pid;
Imu imu;
#if defined(USE_MPL3115A2)
Alt alt;
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
  // Initialize IMU communication
  imu.init();
  delay(10);
#if defined(USE_MPL3115A2)
  // Initialize Baro communication
  alt.init();
  delay(10);
#endif
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
  State packet = rx.getPacket();
#if defined(USE_MPL3115A2)
  // debug(alt.getAlt());
  if (alt.altLocked)
  {
    packet.thrust = pid.lockAlt(packet.thrust);
  }
#endif
  pid.setDesiredState(packet); // convert raw commands to normalized values based on saturated control limits
  Commands commands = pid.control(agImu);
  if (packet.thrust < 10)
  {
    commands = 125;
  }
#if defined(ESC_PROGRAM_MODE)
  if (packet.thrust > 50)
  {
    commands = 250;
  }
#endif
  esc.setSpeed(commands);
  // debug(timer.delta * 1000000);
  // Regulate loop rate
  loopRate(2000); // do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

void loopRate(int freq)
{
  // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  // Sit in loop until appropriate time has passed
  while (invFreq > (checker - timer.now))
  {
    checker = micros();
  }
}

void loopBlink()
{
  // DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
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