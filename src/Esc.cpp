#include "config.h"
#define USE_PMW 1

void Esc::armPmw()
{
    esc1.attach(m1Pin, 900, 2100);
    esc2.attach(m2Pin, 900, 2100);
    esc3.attach(m3Pin, 900, 2100);
    esc4.attach(m4Pin, 900, 2100);
    esc1.write(0);
    esc2.write(0);
    esc3.write(0);
    esc4.write(0);
    delay(1000);
    esc1.write(180);
    esc2.write(180);
    esc3.write(180);
    esc4.write(180);
    delay(100);
}

void Esc::oneShotSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4)
{
    //DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
    /*
   * My crude implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin). The pulselengths being
   * sent are mX_command_PWM, computed in scaleCommands(). This may be replaced by something more efficient in the future.
   */
    uint8_t wentLow = 0;
    uint32_t pulseStart, timer;
    uint8_t flagM1 = 0;
    uint8_t flagM2 = 0;
    uint8_t flagM3 = 0;
    uint8_t flagM4 = 0;

    //Write all motor pins high
    digitalWrite(m1Pin, HIGH);
    digitalWrite(m2Pin, HIGH);
    digitalWrite(m3Pin, HIGH);
    digitalWrite(m4Pin, HIGH);
    pulseStart = micros();

    //Write each motor pin low as correct pulse length is reached
    while (wentLow < 4)
    { //keep going until final (6th) pulse is finished, then done
        timer = micros();
        if ((125 <= timer - pulseStart) && (flagM1 == 0))
        {
            digitalWrite(m1Pin, LOW);
            wentLow = wentLow + 1;
            flagM1 = 1;
        }
        if ((125 <= timer - pulseStart) && (flagM2 == 0))
        {
            digitalWrite(m2Pin, LOW);
            wentLow = wentLow + 1;
            flagM2 = 1;
        }
        if ((125 <= timer - pulseStart) && (flagM3 == 0))
        {
            digitalWrite(m3Pin, LOW);
            wentLow = wentLow + 1;
            flagM3 = 1;
        }
        if ((125 <= timer - pulseStart) && (flagM4 == 0))
        {
            digitalWrite(m4Pin, LOW);
            wentLow = wentLow + 1;
            flagM4 = 1;
        }
    }
}

void Esc::arm()
{
    Serial.print(F("arming motors"));
    pinMode(m1Pin, OUTPUT);
    pinMode(m2Pin, OUTPUT);
    pinMode(m3Pin, OUTPUT);
    pinMode(m4Pin, OUTPUT);
#if defined USE_PMW
    armPmw();
#elif defined USE_ONESHOT
    oneShotSpeed(125, 125, 125, 125);
    delay(1000);
    oneShotSpeed(125, 125, 125, 125);
    delay(1000);
#else
#error no esc protocol defined
#endif
}

void Esc::setSpeed(uint16_t r1, uint16_t r2, uint16_t r3, uint16_t r4)
{
#if defined USE_PMW
    esc1.write(map(r1, 0, 100, 30, 180));
    esc2.write(map(r2, 0, 100, 30, 180));
    esc3.write(map(r3, 0, 100, 30, 180));
    esc4.write(map(r4, 0, 100, 30, 180));
#elif defined USE_ONESHOT
    oneShotSpeed(
        map(r1, 0, 100, 125, 250),
        map(r2, 0, 100, 125, 250),
        map(r3, 0, 100, 125, 250),
        map(r4, 0, 100, 125, 250));
#endif
}