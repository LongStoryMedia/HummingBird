#define DEBUG false
#include "Esc.h"

/* motor layout
        +
      pitch
     |3| |1|
       \ /
     - roll +
       / \
     |4| |2|
      pitch
        -
*/

Esc esc;
Mpu mpu;
Pid pid;

void deserialize(StaticJsonDocument<96> orientation);

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
    StaticJsonDocument<96> orientation; //Create a JsonDocument object
    deserialize(orientation);
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
  Serial.print("\t target-ypr:\t");
  Serial.print(esc.yaw);
  Serial.print("\t");
  Serial.print(esc.pitch);
  Serial.print("\t");
  Serial.print(esc.roll);
  Serial.print("\t");
  Serial.print(esc.thrust);
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

void deserialize(StaticJsonDocument<96> orientation)
{
  DeserializationError error = deserializeJson(orientation, Serial1); //Deserialize JSON data

  if (orientation.isNull())
  {
    return;
  }

  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  esc.setSpeed(orientation, mpu, pid);
}