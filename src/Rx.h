#ifndef RX_H
#define RX_H
#define RC 1

#if defined TEENSY40
#define CE 10
#define CSN 8
#elif defined NANOBLE33
#define CE D10
#define CSN D9
#endif

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
    RF24 radio = RF24(CE, CSN);
    static const uint8_t radioNumber = 0;
    static const uint8_t controllerRadioNumber = 1;
    static const uint8_t packetSize = sizeof(Packet);
#endif
    Packet getBle();
    Packet getRc();
    Packet getLora();
    void initRc();
    void initLora();
    uint32_t rxt;
    Packet packet;

public:
    Packet getPacket();
    void init();
};

#endif