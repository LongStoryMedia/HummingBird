#ifndef ESC_H
#define ESC_H

class Esc
{
private:
#if defined(USE_PWM)
    Servo m1;
    Servo m2;
    Servo m3;
    Servo m4;
#endif
public:
    void arm();
    void setSpeed(Commands commands);
};

#endif