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

void setup()
{
  Serial.begin(9600);
  esc.arm();
  pid.setCoefficients(1.25, 0.075);
  rx.setDisconnectedThreshold(15);
#if !defined(LSM9DS1)
  mpu.calibrate();
#endif
  Serial1.begin(38400);
}

void loop()
{
  mpu.setSpace();
  pid.processTick(mpu.ypr[mpu.pitch], mpu.ypr[mpu.roll]);
  esc.setSpeed(pid.r1, pid.r2, pid.r3, pid.r4);
  rx.parsePacket();
  pid.setTargets(rx.doc["pitch"], rx.doc["roll"], rx.doc["thrust"]);
  Serial1.flush();
#if DEBUG
  Serial.print("ypr\t");
  Serial.print(mpu.ypr[mpu.yaw]);
  Serial.print("\t");
  Serial.print(mpu.ypr[mpu.pitch]);
  Serial.print("\t");
  Serial.print(mpu.ypr[mpu.roll]);
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
}