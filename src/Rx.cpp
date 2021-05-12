#include "config.h"
#define RC 1
#define BLE false
#include "printf.h"

#if defined RC
uint8_t address[][6] = {"bird", "nest"};
#endif

void Rx::init()
{
#if defined RC
    initRc();
#elif defined LORA
    initLora();
#endif
}

Packet Rx::getPacket()
{
#if defined RC
    return getRc();
#elif defined LORA
#else
#error no comm protocol defined
#endif
}

/** --------- BLE ----------- */
#if BLE

Packet Rx::getBle()
{

    if (Serial1.available() > 0)
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
            doc = yprt;
        }
    }
}

#endif

/** --------- RC ----------- */

#if defined RC

void Rx::initRc()
{
    // initialize the transceiver on the SPI bus
    radio.begin();
    // radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit
    radio.setPayloadSize(packetSize); // default value is the maximum 32 bytes

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]); // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(controllerRadioNumber, address[controllerRadioNumber]); // using pipe 1

    radio.startListening(); // put radio in RX mode

    // For debugging info
    // printf_begin();             // needed only once for printing details
    // radio.printDetails();       // (smaller) function that prints raw register values
    // radio.printPrettyDetails(); // (larger) function that prints human readable data
    rxt = micros();
}

Packet Rx::getRc()
{
    uint32_t loopTime = micros();
    // radio.flush_rx();

    if (loopTime > rxt + 2500000)
    {
        radio.failureDetected = true;
        packet.pitch = 0;
        packet.roll = 0;
        packet.yaw = 0;
        packet.thrust = 0;
    }

    if (radio.failureDetected)
    {
        radio.failureDetected = false;
        Serial.println(F("Radio failure detected, restarting radio"));
        radio.powerDown();
        delay(250);
        radio.powerUp();
        radio.flush_rx();
        initRc();
    }

    if (radio.available())
    {
        radio.read(&packet, packetSize);
        rxt = loopTime;
    }
    return packet;
}

#endif