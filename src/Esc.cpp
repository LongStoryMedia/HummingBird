#include "config.h"

// DESCRIPTION: Send pulses to motor pins, oneshot125 protocol
/*
 * Implimentation of OneShot125 protocol which sends 125 - 250us pulses to the ESCs (mXPin).
 */
void Esc::setSpeed(Commands commands)
{
    int wentLow = 0;
    int pulseStart, timer;
    int flagM1 = 0;
    int flagM2 = 0;
    int flagM3 = 0;
    int flagM4 = 0;

    // Write all motor pins high
    digitalWrite(M1_PIN, HIGH);
    digitalWrite(M2_PIN, HIGH);
    digitalWrite(M3_PIN, HIGH);
    digitalWrite(M4_PIN, HIGH);
    pulseStart = micros();

    Serial.print(F("\tm1_command: "));
    Serial.print(commands.m1);
    Serial.print(F("\tm2_command: "));
    Serial.print(commands.m2);
    Serial.print(F("\tm3_command: "));
    Serial.print(commands.m3);
    Serial.print(F("\tm4_command: "));
    Serial.println(commands.m4);

    // Write each motor pin low as correct pulse length is reached
    while (wentLow < 4)
    {
        timer = micros();
        if ((commands.m1 <= timer - pulseStart) && (flagM1 == 0))
        {
            digitalWrite(M1_PIN, LOW);
            wentLow = wentLow + 1;
            flagM1 = 1;
        }
        if ((commands.m2 <= timer - pulseStart) && (flagM2 == 0))
        {
            digitalWrite(M2_PIN, LOW);
            wentLow = wentLow + 1;
            flagM2 = 1;
        }
        if ((commands.m3 <= timer - pulseStart) && (flagM3 == 0))
        {
            digitalWrite(M3_PIN, LOW);
            wentLow = wentLow + 1;
            flagM3 = 1;
        }
        if ((commands.m4 <= timer - pulseStart) && (flagM4 == 0))
        {
            digitalWrite(M4_PIN, LOW);
            wentLow = wentLow + 1;
            flagM4 = 1;
        }
    }
}

void Esc::arm()
{
    Serial.print(F("arming motors"));
    Commands commands;
    commands.m1 = 125;
    commands.m2 = 125;
    commands.m3 = 125;
    commands.m4 = 125;
    setSpeed(commands);
}