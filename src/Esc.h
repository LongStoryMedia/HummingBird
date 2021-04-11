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
    const uint8_t ESC1 = 2;
    const uint8_t ESC2 = 3;
    const uint8_t ESC3 = 4;
    const uint8_t ESC4 = 5;
#endif
#if XIAO
    const uint8_t ESC1 = 1;
    const uint8_t ESC2 = 2;
    const uint8_t ESC3 = 3;
    const uint8_t ESC4 = 10;
#endif
#if defined Servo
    void rm(uint8_t e, Servo esc);
#else
    void rm(uint8_t e, PWMServo esc);
#endif

public:
    void arm();
    void setSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4);
};

#endif