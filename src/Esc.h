#ifndef ESC_H
#define ESC_H

#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>
const size_t ROTOR_NUM = JSON_OBJECT_SIZE(8);

class Esc
{
private:
    Servo esc1;
    Servo esc2;
    Servo esc3;
    Servo esc4;
    //pins
    const uint8_t ESC1 = 10;
    const uint8_t ESC2 = 1;
    const uint8_t ESC3 = 2;
    const uint8_t ESC4 = 3;

    void rm(uint8_t e, Servo esc);

public:
    void arm();
    void setSpeed(StaticJsonDocument<ROTOR_NUM> speed);
    int32_t rotors[4];
    enum ESC
    {
        e1,
        e2,
        e3,
        e4
    };
    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    int16_t thrust;
};

#endif