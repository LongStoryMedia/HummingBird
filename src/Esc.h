#ifndef ESC_H
#define ESC_H

#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>
const size_t ROTOR_NUM = JSON_ARRAY_SIZE(4);

class Esc
{
private:
    Servo esc1;
    Servo esc2;
    Servo esc3;
    Servo esc4;
    const uint8_t ESC1 = 1;
    const uint8_t ESC2 = 2;
    const uint8_t ESC3 = 3;
    const uint8_t ESC4 = 4;

public:
    void arm();
    void setSpeed(StaticJsonDocument<ROTOR_NUM> speed);
    enum ESC
    {
        e1,
        e2,
        e3,
        e4
    };
};

#endif