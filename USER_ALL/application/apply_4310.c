#include "apply_4310.h"

int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
        /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset;
    int max_value;

    if (span <= 0.0f)
    {
        return 0;
    }

    if (x_float < x_min)
    {
        x_float = x_min;
    }
    else if (x_float > x_max)
    {
        x_float = x_max;
    }

    offset = x_min;
    max_value = (1 << bits) - 1;
    return (int)((x_float - offset) * (float)max_value / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

