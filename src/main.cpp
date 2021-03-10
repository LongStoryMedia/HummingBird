#define DEBUG true
// for some reason this needs to be first - namespacing issues with deps I think
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

void setup()
{
  esc.arm();
  pid.setCoefficients(1, 0.1);

  Wire.begin();
  Wire.setClock(400000L);
  mpu.calibrate();

  Serial1.begin(115200);
  Serial.begin(115200);
}

void loop()
{
  mpu.setSpace();
  if (Serial1.available() > 0)
  {
    StaticJsonDocument<ROTOR_NUM> orientation;                          //Create a JsonDocument object
    DeserializationError error = deserializeJson(orientation, Serial1); //Deserialize JSON data

    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }

    esc.setSpeed(orientation, mpu, pid);
  }

#if DEBUG
  Serial.print("ypr\t");
  Serial.print(mpu.ypr[mpu.yaw]);
  Serial.print("\t");
  Serial.print(mpu.ypr[mpu.pitch]);
  Serial.print("\t");
  Serial.print(mpu.ypr[mpu.roll]);
  Serial.print("\t target-ypr:\t");
  Serial.print(esc.roll);
  Serial.print("\t");
  Serial.print(esc.pitch);
  Serial.print("\t");
  Serial.print(esc.yaw);
  Serial.print("\t");
  Serial.print(esc.thrust);
  Serial.print("\t");
  Serial.println(' ');
#endif
}
