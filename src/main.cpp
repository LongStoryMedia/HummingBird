#include "config.h"
#include <Servo.h>

//=============================================================================================================//
//                                                 GLOBALS                                                     //
//=============================================================================================================//

Timer fcTimer(5000, 50);
Timer radioTimer(250, 100);
Input packet;
volatile State state;
volatile State prevState;
Rx rx;
Esc esc;
Pid pid;
Imu imu;
#if defined(USE_ALT)
Alt alt;
Timer altTimer(250, 150);
#endif
#if defined(USE_PROXIMITY_DETECTION)
Proximity proximity;
#endif
Servo cameraServo;

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

//=============================================================================================================//
//                                                 SETUP                                                       //
//=============================================================================================================//

/**
 * @brief core flight logic
 *
 */
void flightLoop();
/**
 * @brief gets radio commands and scales for control loop
 *
 */
void radioLoop();
/**
 * @brief gets baro data and applies to control loops
 *
 */
void altLoop();

void setup()
{
  Serial.begin(5000000); // usb serial
  pinMode(13, OUTPUT);   // pin 13 LED blinker on board, do not modify
  digitalWrite(13, HIGH);

  ALT_WIRE.begin();
  ALT_WIRE.setClock(1000000);
  IMU_WIRE.begin();
  IMU_WIRE.setClock(1000000);

  pid.init();
  rx.init();
#if defined(USE_PROXIMITY_DETECTION)
  proximity.init();
#endif
  imu.init(&IMU_WIRE); // Initialize IMU communication
  imu.calibrate();     // helps to warm up IMU and Madgwick filter before finally entering main loop
#if defined(USE_ALT)
  alt.init(&ALT_WIRE);
#endif
  esc.arm();
  // run loops
  fcTimer.regulate(flightLoop);
  radioTimer.regulate(radioLoop);
  altTimer.regulate(altLoop);
}

void loop()
{
}

void radioLoop()
{
  radioTimer.update();
  packet = rx.getPacket();
  pid.setState(packet); // convert raw commands to normalized values based on saturated control limits
}

void flightLoop()
{
  fcTimer.update();
  Commands commands;
  if (packet.thrust > 10)
  {
    AccelGyro ag = imu.getImu();
    commands = pid.control(ag);
  }
  else
  {
    commands = COMMANDS_LOW;
  }
  esc.setSpeed(commands);
}

void altLoop()
{
  altTimer.update();
#if defined(USE_ALT)
  alt.getAlt();
#endif
#if defined(USE_USS_ALT)
  alt.getAltUss();
#endif
}

//=========================================================================================//
//                                HELPER FUNCTIONS                                         //
//=========================================================================================//

float invSqrt(float x)
{
  // Fast inverse sqrt for madgwick filter
  // float halfx = 0.5f * x;
  // float y = x;
  // long i = *(long *)&y;
  // i = 0x5f3759df - (i >> 1);
  // y = *(float *)&i;
  // y = y * (1.5f - (halfx * y * y));
  // y = y * (1.5f - (halfx * y * y));
  // return y;
  // alternate form:
  unsigned int i = 0x5F1F1412 - (*(unsigned int *)&x >> 1);
  float tmp = *(float *)&i;
  float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
  return y;
}

int multiplyFast(int a, int b) // 5 5
{
  int n1 = abs(a), n2 = abs(b), result = 0;
  bool neg = false;

  if (min(a, b) < 0 && max(a, b) >= 0)
  {
    neg = true;
  }

  while (n2 > 0)
  {
    if ((n2 & 1) == 1) // 5 & 1 == 1, 2 & 1 != 1, 1 & 1 == 1
    {
      result += n1; // 0 + 5 = 5, 5 + 20 = 25
    }
    n2 >>= 1; // 2, 1, 0 (exit condition)
    n1 <<= 1; // 10, 20
  }

  if (neg)
  {
    return (~(result) + 1);
  }
  else
  {
    return result;
  }
}
