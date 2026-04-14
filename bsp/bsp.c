#include "bsp.h"
#include <stdint.h>

uint8_t clamp_uint8(uint8_t value, uint8_t min, uint8_t max)
{
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}

uint16_t clamp_uint16(uint16_t value, uint16_t min, uint16_t max)
{
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}

float clamp_float(float value, float min, float max)
{
    if (value < min) return min;
    else if (value > max) return max;
    else return value;
}