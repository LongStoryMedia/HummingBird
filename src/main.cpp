#include "config.h"

//========================================================================================================================//
//                                                 GLOBALS                                                                //
//========================================================================================================================//

// uint32_t print_counter, serial_counter;
// uint32_t blink_counter, blink_delay;

unsigned long print_counter;
bool blinkAlternate;
Timer timer(5000);
Timer timerOl(100);
State packet;
Rx rx;
Esc esc;
Pid pid;
Imu imu;
#if defined(USE_ALT)
Alt alt;
#endif
#if defined(USE_PROXIMITY_DETECTION)
Proximity proximity;
#endif

#if (PROP_CONFIG == 0)
Prop __p1{Prop::clockwise, Prop::positive, Prop::negative};
Prop __p2{Prop::counterClockwise, Prop::negative, Prop::negative};
Prop __p3{Prop::clockwise, Prop::negative, Prop::positive};
Prop __p4{Prop::counterClockwise, Prop::positive, Prop::positive};
PropConfig propConfig{__p1, __p2, __p3, __p4};
#elif (PROP_CONFIG == 1)
Prop __p1{Prop::counterClockwise, Prop::positive, Prop::negative};
Prop __p2{Prop::clockwise, Prop::negative, Prop::negative};
Prop __p3{Prop::counterClockwise, Prop::negative, Prop::positive};
Prop __p4{Prop::clockwise, Prop::positive, Prop::positive};
PropConfig propConfig{__p1, __p2, __p3, __p4};
#endif

//========================================================================================================================//
//                                                 SETUP                                                                  //
//========================================================================================================================//
/**
 * @brief frequently updated
 *
 */
void innerLoop();
/**
 * @brief periodically updated
 *
 */
void outerLoop();

void setup()
{
  Serial.begin(5000000); // usb serial
  pinMode(13, OUTPUT);   // pin 13 LED blinker on board, do not modify

  // Set built in LED to turn on to signal startup & not to disturb vehicle during IMU calibration
  digitalWrite(13, HIGH);

  ALT_WIRE.begin();
  ALT_WIRE.setClock(1000000);
  IMU_WIRE.begin();
  IMU_WIRE.setClock(1000000);

  delay(10);
  pid.init();
  // Initialize radio communication
  rx.init();
#if defined(USE_ALT)
  // Initialize Baro communication
  alt.init(&ALT_WIRE);
  delay(10);
#endif

#if defined(USE_PROXIMITY_DETECTION)
  proximity.init();
  delay(10);
#endif

  // Initialize IMU communication
  imu.init(&IMU_WIRE);
  // Warm up the loop
  Serial.println("calibrating...");
  imu.calibrate(); // helps to warm up IMU and Madgwick filter before finally entering main loop
  Serial.println("done calibrating");
  delay(10);
  esc.arm();
  delay(100);
  // Indicate entering main loop with 3 quick blinks
  setupBlink(3, 160, 70); // numBlinks, upTime (ms), downTime (ms)
}

void loop()
{
  outerLoop();
}

void outerLoop()
{
  loopBlink(); // indicate we are in main loop with short blink every 1.5 seconds
  timerOl.update();

#if defined(USE_ALT)
  alt.getAlt();
#endif

#if defined(USE_PROXIMITY_DETECTION)
  obstacles obs = proximity.scan();
#endif

  packet = rx.getPacket();
  pid.setDesiredState(packet); // convert raw commands to normalized values based on saturated control limits

  timerOl.regulate(innerLoop);
}

void innerLoop()
{
  timer.update();
  Commands commands;
  commands = COMMANDS_LOW;
  if (packet.thrust > 10)
  {
    commands = pid.control(imu.getImu());
  }
  esc.setSpeed(commands);

  timer.regulate();
}

//========================================================================================================================//
//                                                      FUNCTIONS                                                         //
//========================================================================================================================//

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
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
  // alternate form:
  // unsigned int i = 0x5F1F1412 - (*(unsigned int *)&x >> 1);
  // float tmp = *(float *)&i;
  // float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  // return y;
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