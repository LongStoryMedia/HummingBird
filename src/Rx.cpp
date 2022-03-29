#include "config.h"
// #include "printf.h"

uint8_t address[][6] = {"bird", "nest"};

void Rx::init()
{
    initRc();
}

State Rx::getPacket()
{

    uint32_t loopTime = micros();

    if (loopTime > rxt + 250000)
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
    else
    {
        connectionLoss = true;
    }

    // Low-pass the critical commands and update previous values
    float b = 0.2; // lower=slower, higher=noiser
    packet.thrust = (1.0 - b) * prevPacket.thrust + b * packet.thrust;
    packet.roll = (1.0 - b) * prevPacket.roll + b * packet.roll;
    packet.pitch = (1.0 - b) * prevPacket.pitch + b * packet.pitch;
    packet.yaw = (1.0 - b) * prevPacket.yaw + b * packet.yaw;
    prevPacket = packet;
    failSafe();
    return packet;
}

void Rx::initRc()
{
    // initialize the transceiver on the SPI bus
    radio.begin();
    // radio.setDataRate(RF24_2MBPS);
    // radio.setPALevel(RF24_PA_MAX); // RF24_PA_MAX is default.
    // radio.txDelay = 0;
    // radio.disableCRC();
    // radio.setAutoAck(true);
    // radio.disableDynamicPayloads();
    // radio.setAutoAck(false);

    // save on transmission time by setting the radio to only transmit the
    // number of bytes we need to transmit
    radio.setPayloadSize(packetSize); // default value is the maximum 32 bytes

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(address[radioNumber]); // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(controllerRadioNumber, address[controllerRadioNumber]); // using pipe 1
    radio.startListening();                                                       // put radio in RX mode

    // For debugging info
    // printf_begin();             // needed only once for printing details
    // radio.printDetails();       // (smaller) function that prints raw register values
    // radio.printPrettyDetails(); // (larger) function that prints human readable data
    rxt = micros();
}

void Rx::failSafe()
{
    int check1 = 0;
    int check2 = 0;
    int check3 = 0;
    int check4 = 0;

    // Triggers for failure criteria
    if (packet.thrust > 1000 || packet.thrust <= 0)
        check1 = 1;
    if (packet.roll > 500 || packet.roll < -500)
        check2 = 1;
    if (packet.pitch > 500 || packet.pitch < -500)
        check3 = 1;
    if (packet.yaw > 500 || packet.yaw < -500)
        check4 = 1;

    // If any failures, set to default failsafe values
    if ((check1 + check2 + check3 + check4) > 0)
    {
        packet.thrust = 0;
        packet.roll = 0;
        packet.pitch = 0;
        packet.yaw = 0;
    }
}