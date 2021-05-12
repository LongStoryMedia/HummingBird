#ifndef ESC_H
#define ESC_H

class Esc
{
private:
#if TEENSY40
    PWMServo esc1;
    PWMServo esc2;
    PWMServo esc3;
    PWMServo esc4;
#else
    Servo esc1;
    Servo esc2;
    Servo esc3;
    Servo esc4;
#endif
    //pins
#if TEENSY40
    static const uint8_t m1Pin = 2;
    static const uint8_t m2Pin = 3;
    static const uint8_t m3Pin = 4;
    static const uint8_t m4Pin = 5;
#endif
#if XIAO
    static const uint8_t m1Pin = 1;
    static const uint8_t m2Pin = 2;
    static const uint8_t m3Pin = 3;
    static const uint8_t m4Pin = 10;
#endif
#if NANOBLE33
    static const uint8_t m1Pin = 2;
    static const uint8_t m2Pin = 3;
    static const uint8_t m3Pin = 4;
    static const uint8_t m4Pin = 5;
#endif
#if defined Servo
    void armPmw();
    void oneShotSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4);
#else
    void armPmw();
    void oneShotSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4);
#endif

public:
    void arm();
    void setSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4);
};

#endif