#ifndef _BMI088_H_
#define _BMI088_H_

#include "main.h"
#include "spi.h"
#include <stdint.h>

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t gx;
	int16_t gy;
	int16_t gz;
} BMI088_RawData_t;

typedef enum
{
	BMI088_OK = 0,
	BMI088_ERR_ACC_CHIP_ID,
	BMI088_ERR_GYRO_CHIP_ID,
	BMI088_ERR_SPI,
} BMI088_Status_e;

BMI088_Status_e BMI088_Init(void);

#endif