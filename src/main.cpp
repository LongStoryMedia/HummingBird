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

uint32_t t;
void loopRate(int freq);

void setup()
{
  Serial.begin(9600);
  // esc.arm();
  pid.setCoefficients(1.50, 0.001, 0.001);
#if !defined(LSM9DS1)
  mpu.calibrate();
#endif
  rx.init();
  Serial1.begin(38400);
}

void loop()
{
  t = micros();
  mpu.setSpace();
  rx.parsePacket();
  pid.setTargets(rx.packet.pitch, rx.packet.roll, rx.packet.thrust);
  pid.processTick(mpu.ypr[mpu.pitch], mpu.ypr[mpu.roll]);
  pid.deriveError(mpu.gx, mpu.gy);
  esc.setSpeed(pid.r1, pid.r2, pid.r3, pid.r4);
#if DEBUG
  Serial.print("rx\t");
  Serial.print(rx.packet.yaw);
  Serial.print("\t");
  Serial.print(rx.packet.pitch);
  Serial.print("\t");
  Serial.print(rx.packet.roll);
  Serial.print("\t");
  Serial.print(rx.packet.thrust);
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
  loopRate(200000); //do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
  // delay(500);
  Serial1.flush();
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