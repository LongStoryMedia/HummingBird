#include "config.h"

#if !defined(USE_PROXIMITY_DETECTION)
#include "Proximity.h"
#define SENSOR_LEFT 1000
#define SENSOR_RIGHT 1000
#define SENSOR_REAR 1000
#define SENSOR_FRONT_LEFT 1000
#define SENSOR_FRONT_RIGHT 1000
#endif

void Proximity::init()
{
    pinMode(SENSOR_LEFT, INPUT);
    pinMode(SENSOR_RIGHT, INPUT);
    pinMode(SENSOR_REAR, INPUT);
    pinMode(SENSOR_FRONT_LEFT, INPUT);
    pinMode(SENSOR_FRONT_RIGHT, INPUT);
    _obstacles.clear();
}

obstacles Proximity::scan()
{
    _obstacles.clear();
    _obstacles.left = digitalReadFast(SENSOR_LEFT);
    _obstacles.right = digitalReadFast(SENSOR_RIGHT);
    _obstacles.rear = digitalReadFast(SENSOR_REAR);
    _obstacles.frontLeft = digitalReadFast(SENSOR_FRONT_LEFT);
    _obstacles.frontRight = digitalReadFast(SENSOR_FRONT_RIGHT);
    return _obstacles;
}