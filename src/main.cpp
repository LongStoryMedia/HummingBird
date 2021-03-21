#define DEBUG false
#include "Esc.h"

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

Esc esc;
Mpu mpu;
Pid pid;

void setup()
{
#if DEBUG
  Serial.begin(38400);
#endif

  esc.arm();
  pid.setCoefficients(1.0, 0.01);

  Wire.begin();
  Wire.setClock(400000L);
  mpu.calibrate();

  Serial1.begin(38400);
  Serial1.flush();
}

void loop()
{
  if (Serial1.available() > 0)
  {
    StaticJsonDocument<YPRT> yprt;                               //Create a JsonDocument object
    DeserializationError error = deserializeJson(yprt, Serial1); //Deserialize JSON data

    if (error)
    {
#if DEBUG
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
#endif
      return;
    }

    pid.processTick(mpu.ypr[mpu.pitch], mpu.ypr[mpu.roll], yprt["pitch"], yprt["roll"], yprt["thrust"]);

    esc.setSpeed(pid.r1, pid.r2, pid.r3, pid.r4);
  }
  mpu.setSpace();
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