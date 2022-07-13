#ifndef FILTER_LOCAL_H
#define FILTER_LOCAL_H

class Filter
{
private:
    /* data */
public:
    template <class T>
    void lowPass(T prevVal, T currVal, float filterParam);
};

#endif