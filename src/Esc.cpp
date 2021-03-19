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

void Esc::setSpeed(StaticJsonDocument<96> speed, Mpu mpu, Pid pid)
{
    roll = speed["roll"];
    pitch = speed["pitch"];
    yaw = speed["yaw"];
    thrust = speed["thrust"];
    pid.setPitchAndRoll(mpu.ypr[mpu.pitch], mpu.ypr[mpu.roll]);
    pid.processTick(pitch, roll, thrust);
    esc1.writeMicroseconds(pid.r1);
    esc2.writeMicroseconds(pid.r2);
    esc3.writeMicroseconds(pid.r3);
    esc4.writeMicroseconds(pid.r4);
}