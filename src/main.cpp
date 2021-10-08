#include "config.h"

Esc esc;
Pid pid;
Mpu mpu;
Rx rx;
/* motor layout
        -
      pitch
     |1| |3|
       \ /
     + roll -
       / \
     |2| |4|
      pitch
        +
*/

uint32_t pt, t, dt, blinkCounter, blinkDelay, startTime, blinkStart, loopEnd, loopRateEnd;
bool blinkAlternate;

void loopRate(int freq);
void loopBlink();
void setupBlink(int numBlinks, int upTime, int downTime);

void setup()
{
  Serial.begin(500000);
  pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify
  esc.arm();
  pid.setCoefficients(KP_ROLL, KP_PITCH, KP_YAW, KI_ROLL, KI_PITCH, KI_YAW, KD_ROLL, KD_PITCH, KD_YAW, I_LIMIT);
  mpu.calibrate();
  rx.init();
  setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
  startTime = micros();
}

void loop()
{
  pt = t;
  t = micros();
  dt = t - pt;
  loopBlink();
#if DEBUG_HZ
  Serial.print("\tdelta time: ");
  Serial.print(dt);
  Serial.print("\ttime in loop rate: ");
  Serial.print(t - loopEnd);
  Serial.print("\tloop speed: ");
  Serial.println(dt - (t - loopEnd));
#endif
  Orientation orientation = mpu.getOrientation(dt);
  Packet packet = rx.getPacket();

  pid.setTargets(packet.yaw, packet.pitch, packet.roll, packet.thrust);
  pid.processTick(orientation.yaw, orientation.pitch, orientation.roll, orientation.gx, orientation.gy, orientation.gz, dt);

  // // // wait for imu to calibrate
  if (t > startTime + 30000000)
  {
    esc.setSpeed(pid.r1, pid.r2, pid.r3, pid.r4);
  }

#if DEBUG
  Serial.print("orientation: \t");
  Serial.print(orientation.yaw);
  Serial.print("\t");
  Serial.print(orientation.pitch);
  Serial.print("\t");
  Serial.print(orientation.roll);
  Serial.print("\t rx: \t");
  Serial.print(packet.yaw);
  Serial.print("\t");
  Serial.print(packet.pitch);
  Serial.print("\t");
  Serial.print(packet.roll);
  Serial.print("\t");
  Serial.print(packet.thrust);
  Serial.print("\t motor speed:\t");
  Serial.print(pid.r1);
  Serial.print("\t");
  Serial.print(pid.r2);
  Serial.print("\t");
  Serial.print(pid.r3);
  Serial.print("\t");
  Serial.print(pid.r4);
  Serial.println(' ');
#endif
  loopEnd = micros();
  loopRate(2000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

void loopRate(int freq)
{
  float invFreq = 1.0 / freq * 1000000.0;
  uint32_t checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - t))
  {
    checker = micros();
  }
  loopRateEnd = micros();
}

void loopBlink()
{
  if (t - blinkCounter > blinkDelay)
  {
    blinkCounter = micros();
    digitalWrite(13, blinkAlternate); //pin 13 is built in LED

    if (blinkAlternate == 1)
    {
      blinkAlternate = 0;
      blinkDelay = 100000;
    }
    else if (blinkAlternate == 0)
    {
      blinkAlternate = 1;
      blinkDelay = 2000000;
    }
  }
}

void setupBlink(int numBlinks, int upTime, int downTime)
{
  for (int j = 1; j <= numBlinks; j++)
  {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}