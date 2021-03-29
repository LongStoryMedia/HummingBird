#ifndef ESC_H
#define ESC_H

class Esc
{
private:
    Servo esc1;
    Servo esc2;
    Servo esc3;
    Servo esc4;
    //pins
    const uint8_t ESC1 = 1;
    const uint8_t ESC2 = 2;
    const uint8_t ESC3 = 3;
    const uint8_t ESC4 = 10;

    void rm(uint8_t e, Servo esc);

public:
    void arm();
    void setSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4);
};

#endif