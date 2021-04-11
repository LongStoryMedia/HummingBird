#ifndef RX_H
#define RX_H

typedef struct
{
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    uint8_t thrust;
} Packet;

class Rx
{
private:
#if defined RC
    RF24 radio = RF24(10, 8);
    static const uint8_t radioNumber = 0;
    static const uint8_t controllerRadioNumber = 1;
    static const uint8_t packetSize = sizeof(Packet);
#endif
    void getBle();
    void getRc();
    void getLora();
    void initRc();
    void initLora();
    uint32_t rxt;

public:
    void parsePacket();
    void init();
    Packet packet;
};

#endif