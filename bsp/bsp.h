#ifndef _BSP_H_
#define _BSP_H_

// #define REGISTER(T) \
//     T *T##_instance = (T*)malloc(sizeof(T)); \
//     if memset(T_##instance, 0, sizeof(T))
// #endif

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/types.h>

#include "FreeRTOS.h"
#include "main.h"
#include "bsp_can.h"
#include "dwt.h"
#include "pid.h"
#include "motor.h"
// #include "chassis.h"
// #include "task.h"

uint8_t clamp_uint8(uint8_t value, uint8_t min, uint8_t max);
uint16_t clamp_uint16(uint16_t value, uint16_t min, uint16_t max);
float clamp_float(float value, float min, float max);

#endif