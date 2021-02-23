#define DEBUG true

#include "Esc.h"
#include "Mpu.h"

Mpu mpu;
Esc esc;

uint16_t *space;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  esc.arm();
  Serial1.flush();
  mpu.wire();
}

void loop()
{
  mpu.setSpace();
  space = mpu.getSpace();

#if DEBUG
  Serial.print("Accelerometer: ");
  Serial.print("X = ");
  Serial.print(space[mpu.AcX]);
  Serial.print(" | Y = ");
  Serial.print(space[mpu.AcY]);
  Serial.print(" | Z = ");
  Serial.println(space[mpu.AcZ]);
  Serial.print("Gyroscope: ");
  Serial.print("X = ");
  Serial.print(space[mpu.GyX]);
  Serial.print(" | Y = ");
  Serial.print(space[mpu.GyY]);
  Serial.print(" | Z = ");
  Serial.println(space[mpu.GyZ]);
  Serial.println(" ");
#endif
  if (Serial1.available() > 0)
  {
    StaticJsonDocument<ROTOR_NUM> speed;                          //Create a JsonDocument object
    DeserializationError error = deserializeJson(speed, Serial1); //Deserialize JSON data

    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      Serial1.print(F("deserializeJson() failed: "));
      Serial1.println(error.f_str());
      return;
    }

    esc.setSpeed(speed);

#if DEBUG
    Serial.print(" | e1: ");
    Serial.print(speed[esc.e1].as<uint32_t>());
    Serial.print(" | e2: ");
    Serial.print(speed[esc.e2].as<uint32_t>());
    Serial.print(" | e3: ");
    Serial.print(speed[esc.e3].as<uint32_t>());
    Serial.print(" | e4: ");
    Serial.print(speed[esc.e4].as<uint32_t>());
    Serial.print(" | ");
    Serial.println(' ');
#endif
  }
  delay(500);
}
