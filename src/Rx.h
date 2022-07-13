#ifndef RX_H
#define RX_H

class Rx
{
private:
    RF24 radio = RF24(CE_NRF24, CSN_NRF24);
    static const uint8_t radioNumber = 0;
    static const uint8_t controllerRadioNumber = 1;
    uint8_t packetSize = sizeof(Input);
    Input getBle();
    Input getRc();
    Input getLora();
    void initRc();
    void initLora();
    uint32_t rxt;
    void failSafe();
    Input packet;
    Input prevPacket;
    Filter filter;

public:
    Input getPacket();
    void init();
    bool connectionLoss;
};

#endif