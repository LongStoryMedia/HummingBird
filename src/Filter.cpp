#include "config.h"

template <class T>
void Filter::lowPass(T prevVal, T currVal, float filterParam)
{
    if (prevVal == 0)
    {
        prevVal = currVal;
    }

    // only integrate on change
    if (currVal != prevVal)
    {
        // LP filter alt data
        currVal = (prevVal * (1.0 - filterParam)) + (currVal * filterParam);
    }
    prevVal = currVal;
}