#include "config.h"

void Rx::parsePacket()
{
    if (Serial1.available() > 0)
    {
        connected = true;
        unavailableIterations = 0;
        deserialize();
        // pid.setTargets(yprt["pitch"], yprt["roll"], yprt["thrust"]);
    }
    else
    {
        unavailableIterations += 1;
    }

    if (unavailableIterations >= disconnectedThreshold)
    {
        StaticJsonDocument<0> emptyDoc;
        doc = emptyDoc.to<JsonVariant>();
    }
}

void Rx::deserialize()
{
    StaticJsonDocument<YPRT> yprt;
    DeserializationError error = deserializeJson(yprt, Serial1); //Deserialize JSON data

    if (error)
    {
#if DEBUG
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
#endif
    }
    else
    {
        doc = yprt.to<JsonVariant>();
    }
}

void Rx::setDisconnectedThreshold(uint8_t threshold)
{
    disconnectedThreshold = threshold;
}