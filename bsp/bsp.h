#ifndef _BSP_H_
#define _BSP_H_

// #define REGISTER(T) \
//     T *T##_instance = (T*)malloc(sizeof(T)); \
//     if memset(T_##instance, 0, sizeof(T))
// #endif

#include <stdint.h>
uint8_t LimitValue_uint8(uint8_t value, uint8_t min, uint8_t max);
uint16_t LimitValue_uint16(uint16_t value, uint16_t min, uint16_t max);
float LimitValue_float(float value, float min, float max);

#endif