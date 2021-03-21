#include "Esc.h"

void Esc::rm(uint8_t e, Servo esc)
{
    Serial.print(F("arming motor "));
    Serial.println(e);
    esc.attach(e);
    delay(2000);
    esc.writeMicroseconds(1000);
    delay(1000);
    esc.writeMicroseconds(2000);
    delay(8000);
    esc.writeMicroseconds(1000);
    delay(1000);
}

void Esc::arm()
{
    rm(ESC1, esc1);
    rm(ESC2, esc2);
    rm(ESC3, esc3);
    rm(ESC4, esc4);
}

void Esc::setSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4)
{
    esc1.writeMicroseconds(r1);
    esc2.writeMicroseconds(r2);
    esc3.writeMicroseconds(r3);
    esc4.writeMicroseconds(r4);
}