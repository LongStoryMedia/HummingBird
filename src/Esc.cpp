#include "config.h"

void Esc::rm(uint8_t e, PWMServo esc)
{
    Serial.print(F("arming motor "));
    Serial.println(e);
    esc.attach(e);
    esc.write(0);
    delay(1000);
    esc.write(180);
    delay(8000);
    esc.write(0);
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
    esc1.write(r1);
    esc2.write(r2);
    esc3.write(r3);
    esc4.write(r4);
}