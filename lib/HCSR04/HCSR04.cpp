#include "Arduino.h"
#include "HCSR04.h"

HCSR04 *HCSR04::self(NULL);

HCSR04::HCSR04(uint8_t echo, uint8_t trig)
{
    echoPin = echo;
    trigPin = trig;
    if (self == 0)
    {
        self = this;
    }
}

HCSR04::HCSR04(uint8_t echoTrig)
{
    echoPin = echoTrig;
    trigPin = echoTrig;
    if (self == 0)
    {
        self = this;
    }
}

void HCSR04::init()
{
    pinMode(echoPin, INPUT);
    pinMode(trigPin, OUTPUT);
}

void HCSR04::start()
{
    // Clears the trigPin condition
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    isActive = true;
}

void HCSR04::stop()
{
    isActive = false;
}

uint32_t HCSR04::ping()
{
    if (isActive)
    {
        return distance;
    }
}

vfp HCSR04::getInterrupt()
{
    return []() mutable
    {
        HCSR04 *_this = HCSR04::instance();

        // if (_this->sent < _this->recieved)
        // {
        //     return;
        // }

        bool awaitingEcho = digitalRead(_this->echoPin);

        if (awaitingEcho)
        {
            _this->sent = micros();
        }
        else
        {
            _this->recieved = micros();
            _this->distance = (_this->recieved - _this->sent) * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
            _this->start();
        }
    };
}