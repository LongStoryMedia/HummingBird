#ifndef RX_H
#define RX_H
#define RC 1

#define CE 10
#define CSN 9

class Rx
{
private:
    RF24 radio = RF24(CE, CSN);
    static const uint8_t radioNumber = 0;
    static const uint8_t controllerRadioNumber = 1;
    uint8_t packetSize = sizeof(State);
    State getBle();
    State getRc();
    State getLora();
    void initRc();
    void initLora();
    uint32_t rxt;
    State packet;
    State prevPacket;
    void failSafe();

public:
    State getPacket();
    void init();
};

#endif