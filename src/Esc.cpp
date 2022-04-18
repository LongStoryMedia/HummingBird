#include "config.h"

// DESCRIPTION: Send pulses to motor pins
void Esc::setSpeed(Commands commands)
{
    if (commands.m1 > COMMANDS_HIGH ||
        commands.m2 > COMMANDS_HIGH ||
        commands.m3 > COMMANDS_HIGH ||
        commands.m4 > COMMANDS_HIGH)
    {
        commands = COMMANDS_LOW;
    }

    // Serial.print("m1: ");
    // Serial.print(commands.m1);
    // Serial.print(", m2: ");
    // Serial.print(commands.m2);
    // Serial.print(", m3: ");
    // Serial.print(commands.m3);
    // Serial.print(", m4: ");
    // Serial.println(commands.m4);

#if defined(USE_PWM)
    m1.write(commands.m1);
    m2.write(commands.m2);
    m3.write(commands.m3);
    m4.write(commands.m4);
#else
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
#endif
}

void Esc::arm()
{
    Commands commands;
    Serial.print(F("arming motors"));
#if defined(USE_PWM)
    m1.attach(M1_PIN, 900, 2100);
    m2.attach(M2_PIN, 900, 2100);
    m3.attach(M3_PIN, 900, 2100);
    m4.attach(M4_PIN, 900, 2100);
#else
    pinMode(M1_PIN, OUTPUT);
    pinMode(M2_PIN, OUTPUT);
    pinMode(M3_PIN, OUTPUT);
    pinMode(M4_PIN, OUTPUT);
#endif
#if !defined(ESC_PROGRAM_MODE)
    commands = COMMANDS_LOW;
    setSpeed(commands);
#endif
}