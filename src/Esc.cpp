#include "Esc.h"

void Esc::rm(uint8_t e, Servo esc)
{
    Serial.print("arming motor ");
    Serial.println(e);
    Serial1.print("arming motor ");
    Serial1.println(e);
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

void Esc::setSpeed(StaticJsonDocument<ROTOR_NUM> speed)
{
    roll = speed["roll"].as<int16_t>();
    pitch = speed["pitch"].as<int16_t>();
    yaw = speed["yaw"].as<int16_t>();
    thrust = speed["thrust"].as<int16_t>();
    // esc1.writeMicroseconds(speed[e1].as<int32_t>());
    // esc2.writeMicroseconds(speed[e2].as<int32_t>());
    // esc3.writeMicroseconds(speed[e3].as<int32_t>());
    // esc4.writeMicroseconds(speed[e4].as<int32_t>());
}