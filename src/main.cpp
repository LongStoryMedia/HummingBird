#define DEBUG false
// for some reason this needs to be first - namespacing issues with deps I think
#include "Esc.h"
#include "Mpu.h"
#include "Pid.h"
/* motor layout
      FRONT
      3  1
       \/
       /\
      4  2
      BACK
*/

Esc esc;
Mpu mpu;
Pid pitch;
Pid yaw;
Pid roll;

void setup()
{
  Serial1.begin(115200);
  // esc.arm();
  Serial1.flush();

  Wire.begin();
  Wire.setClock(400000L);

  // TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  Serial.begin(9600);
  mpu.calibrate();
}

void loop()
{
  if (Serial1.available() > 0)
  {
    StaticJsonDocument<ROTOR_NUM> orientation;                          //Create a JsonDocument object
    DeserializationError error = deserializeJson(orientation, Serial1); //Deserialize JSON data

    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      // Serial1.print(F("deserializeJson() failed: "));
      // Serial1.println(error.f_str());
    }

    esc.setSpeed(orientation);
  }
  mpu.getSpace();

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
  serializeJson(mpu.doc, Serial1);
  // Serial1.flush();
}
