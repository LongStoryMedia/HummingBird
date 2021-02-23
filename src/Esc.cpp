#include "Esc.h"

void Esc::arm()
{
    esc1.attach(ESC1);
    esc2.attach(ESC2);
    esc3.attach(ESC3);
    esc4.attach(ESC4);
    // arm motor 1
    Serial.println("arming motor 1");
    Serial1.println("arming motor 1");
    esc1.writeMicroseconds(1000);
    delay(1000);
    esc1.writeMicroseconds(2000);
    delay(2000);
    esc1.writeMicroseconds(1000);
    delay(1000);
    // arm motor 2
    Serial.println("arming motor 2");
    Serial1.println("arming motor 2");
    esc2.writeMicroseconds(1000);
    delay(1000);
    esc2.writeMicroseconds(2000);
    delay(2000);
    esc2.writeMicroseconds(1000);
    delay(1000);
    // arm motor 3
    Serial.println("arming motor 3");
    Serial1.println("arming motor 3");
    esc3.writeMicroseconds(1000);
    delay(1000);
    esc3.writeMicroseconds(2000);
    delay(2000);
    esc3.writeMicroseconds(1000);
    delay(1000);
    // arm motor 4
    Serial.println("arming motor 4");
    Serial1.println("arming motor 4");
    esc4.writeMicroseconds(1000);
    delay(1000);
    esc4.writeMicroseconds(2000);
    delay(2000);
    esc4.writeMicroseconds(1000);
    delay(1000);
}

void Esc::setSpeed(StaticJsonDocument<ROTOR_NUM> speed)
{
    esc1.writeMicroseconds(speed[e1].as<uint32_t>());
    esc2.writeMicroseconds(speed[e2].as<uint32_t>());
    esc3.writeMicroseconds(speed[e3].as<uint32_t>());
    esc4.writeMicroseconds(speed[e4].as<uint32_t>());
}