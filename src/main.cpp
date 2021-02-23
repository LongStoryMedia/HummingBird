#define DEBUG true

#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>

const uint8_t esc1 = 1;
const uint8_t esc2 = 2;
const uint8_t esc3 = 3;
const uint8_t esc4 = 4;
const size_t ROTOR_NUM = JSON_ARRAY_SIZE(4);

Servo m1;
Servo m2;
Servo m3;
Servo m4;

uint32_t r1s;
uint32_t r2s;
uint32_t r3s;
uint32_t r4s;

enum ROTORS
{
  r1,
  r2,
  r3,
  r4
} rotors;

void arm();

void setup()
{
  Serial.begin(115200);
  Serial1.begin(115200);
  m1.attach(esc1);
  m2.attach(esc2);
  m3.attach(esc3);
  m4.attach(esc4);
  arm();
  delay(2000);
  Serial1.flush();
}

void loop()
{
  if (Serial1.available() > 0)
  {
    StaticJsonDocument<ROTOR_NUM> speed;                          //Create a JsonDocument object
    DeserializationError error = deserializeJson(speed, Serial1); //Deserialize JSON data
    r1s = speed[r1].as<uint32_t>();
    r2s = speed[r2].as<uint32_t>();
    r3s = speed[r3].as<uint32_t>();
    r4s = speed[r4].as<uint32_t>();

#if DEBUG
    if (error)
    {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
    }
    else
    {
      Serial.print(" | r1: ");
      Serial.print(r1s);
      Serial.print(" | r2: ");
      Serial.print(r2s);
      Serial.print(" | r3: ");
      Serial.print(r3s);
      Serial.print(" | r4: ");
      Serial.print(r4s);
      Serial.print(" | ");
      Serial.println(' ');
    }
#endif
  }
}

void arm()
{
  // arm motor 1
  Serial.println("arming motor 1");
  Serial1.println("arming motor 1");
  m1.writeMicroseconds(1000);
  delay(1000);
  m1.writeMicroseconds(2000);
  delay(2000);
  m1.writeMicroseconds(1000);
  delay(1000);
  // arm motor 2
  Serial.println("arming motor 2");
  Serial1.println("arming motor 2");
  m2.writeMicroseconds(1000);
  delay(1000);
  m2.writeMicroseconds(2000);
  delay(2000);
  m2.writeMicroseconds(1000);
  delay(1000);
  // arm motor 3
  Serial.println("arming motor 3");
  Serial1.println("arming motor 3");
  m3.writeMicroseconds(1000);
  delay(1000);
  m3.writeMicroseconds(2000);
  delay(2000);
  m3.writeMicroseconds(1000);
  delay(1000);
  // arm motor 4
  Serial.println("arming motor 4");
  Serial1.println("arming motor 4");
  m4.writeMicroseconds(1000);
  delay(1000);
  m4.writeMicroseconds(2000);
  delay(2000);
  m4.writeMicroseconds(1000);
  delay(1000);
}