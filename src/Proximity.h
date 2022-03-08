#ifndef PROXIMITY_H
#define PROXIMITY_H

class Proximity
{
private:
    obstacles _obstacles;

public:
    void init();
    obstacles scan();
};

#endif