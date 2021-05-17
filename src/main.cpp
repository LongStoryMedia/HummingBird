#include "config.h"

Esc esc;
Pid pid;
Mpu mpu;
Rx rx;
/* motor layout`
        -
      pitch
     |1| |2|
       \ /
     + roll -
       / \
     |3| |4|
      pitch
        +
*/

uint32_t t, blinkCounter, blinkDelay, startTime, blinkStart;
bool blinkAlternate;

void loopRate(int freq);
void loopBlink();
void setupBlink(int numBlinks, int upTime, int downTime);

void setup()
{
  Serial.begin(500000);
  // pinMode(13, OUTPUT); //pin 13 LED blinker on board, do not modify
  esc.arm();
  pid.setCoefficients(KP, KI, KD);
  // #if !defined(LSM9DS1)
  // #endif
  mpu.calibrate();

  rx.init();
  // Serial1.begin(38400);
  // setupBlink(3, 160, 70); //numBlinks, upTime (ms), downTime (ms)
  startTime = micros();
}

void loop()
{
  // loopBlink();
  t = micros();
  Orientation orientation = mpu.getOrientation();
  Packet packet = rx.getPacket();

  pid.setTargets(packet.pitch, packet.roll, packet.thrust);
  pid.setDerivatives(orientation.gx, orientation.gy);
  pid.processTick((int16_t)orientation.pitch, (int16_t)orientation.roll);

  // wait for mpu to calibrate
  if (t > startTime + 60000000)
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
  loopRate(2000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
  // delay(500);
  // Serial1.flush();
}

void loopRate(int freq)
{
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0 / freq * 1000000.0;
  unsigned long checker = micros();

  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - t))
  {
    checker = micros();
  }
}

void loopBlink()
{
  //DESCRIPTION: Blink LED on board to indicate main loop is running
  /*
   * It looks cool.
   */
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
  //DESCRIPTION: Simple function to make LED on board blink as desired
  for (int j = 1; j <= numBlinks; j++)
  {
    digitalWrite(13, LOW);
    delay(downTime);
    digitalWrite(13, HIGH);
    delay(upTime);
  }
}