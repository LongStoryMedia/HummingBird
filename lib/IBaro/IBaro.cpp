#include "IBaro.h"

bool IBaro::oneShot()
{
    if (lastUpdateTime + clockSpeed <= micros())
    {
        lastUpdateTime = micros();
        return true;
    }
    return false;
}
