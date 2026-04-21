#ifndef _BSP_SPI_H_
#define _BSP_SPI_H

#include "main.h"
#include "stm32f407xx.h"
#include <stdint.h>

#define SPI_MX_REGISTER_CNT 4

typedef enum
{
    SPI_TRANSMIT_MODE_POLLING = 0,
    SPI_TRANSMIT_MODE_INTERRUPT,
    SPI_TRANSMIT_MODE_DMA,
} SPI_TransmitMode_e;

// SPI实例结构体,每个使用SPI的模块都应该有一个这样的实例
typedef struct _
{
    SPI_HandleTypeDef *hspi;
    SPI_TransmitMode_e transmit_mode;

    // uint8_t rx_buff[8];
    // 用于接收数据的GPIO引脚
    GPIO_TypeDef *gpio_port;
    uint16_t gpio_pin;

    void (*spi_module_callback)(struct _ *);
} SPIINstance;

typedef struct
{
    SPI_HandleTypeDef *hspi;
    SPI_TransmitMode_e transmit_mode;
    void (*spi_module_callback)(struct _ *);
} SPI_Init_Config_s;

SPIINstance* SPI_Register(SPI_Init_Config_s *config);
void SPI_Transmit(SPIINstance *instance, uint8_t *data, uint16_t size);
void SPI_Receive(SPIINstance *instance, uint8_t *data, uint16_t size);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

#endif