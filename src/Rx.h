#ifndef RX_H
#define RX_H

class Rx
{
private:
    static const size_t YPRT = JSON_OBJECT_SIZE(8);
    uint8_t unavailableIterations;
    bool connected;
    void deserialize();
    uint8_t disconnectedThreshold;

public:
    void setDisconnectedThreshold(uint8_t threshold);
    void parsePacket();
    JsonVariant doc;
};

#endif