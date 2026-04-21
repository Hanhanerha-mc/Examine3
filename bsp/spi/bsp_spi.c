#include "bsp_spi.h"
#include <stdlib.h>
#include <string.h>

SPIINstance* spi_instances[SPI_MX_REGISTER_CNT] = {NULL}; // SPI实例指针数组,存储实例，用于回调时索引不同实例
static uint8_t spi_idx = 0; // 全局SPI实例索引,每次有新的模块注册会自增

SPIINstance* SPI_Register(SPI_Init_Config_s *config)
{
    if (spi_idx >= SPI_MX_REGISTER_CNT) return NULL; // 错误处理
    SPIINstance *spi_instance = (SPIINstance *)malloc(sizeof(SPIINstance));
    memset(spi_instance, 0, sizeof(SPIINstance));
    spi_instance->hspi = config->hspi;
    spi_instance->transmit_mode = config->transmit_mode;
    spi_instance->spi_module_callback = config->spi_module_callback;

    spi_instances[spi_idx++] = spi_instance; // 将实例保存到spi_instances中
    return spi_instance;
}

void SPI_Transmit(SPIINstance *instance, uint8_t *data, uint16_t size)
{
    if (instance == NULL || data == NULL || size == 0) return; // 错误处理
    switch (instance->transmit_mode)
    {
        case SPI_TRANSMIT_MODE_POLLING:
            HAL_SPI_Transmit(instance->hspi, data, size, 100);
            break;
        case SPI_TRANSMIT_MODE_INTERRUPT:
            HAL_SPI_Transmit_IT(instance->hspi, data, size);
            break;
        case SPI_TRANSMIT_MODE_DMA:
            HAL_SPI_Transmit_DMA(instance->hspi, data, size);
            break;
    }
    return;
}

/*感觉不需要检测模式，全用DMA不香吗，目前先传用框架写法*/
void SPI_Receive(SPIINstance *instance, uint8_t *data, uint16_t size)
{
    if (instance == NULL || data == NULL || size == 0) return; // 错误处理
    switch (instance->transmit_mode)
    {
        case SPI_TRANSMIT_MODE_POLLING:
            HAL_SPI_Receive(instance->hspi, data, size, 100);
            break;
        case SPI_TRANSMIT_MODE_INTERRUPT:
            HAL_SPI_Receive_IT(instance->hspi, data, size);
            break;
        case SPI_TRANSMIT_MODE_DMA:
            HAL_SPI_Receive_DMA(instance->hspi, data, size);
            break;
    }
    return;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) 
{
    for (uint8_t i = 0; i < spi_idx; i++)
    {
        if (spi_instances[i]->hspi == hspi && HAL_GPIO_ReadPin(spi_instances[i]->gpio_port, spi_instances[i]->gpio_pin) == GPIO_PIN_RESET) // 检查SPI实例和片选引脚状态是否为低电平
        //中断传输或DMA传输结束，应该拉高片选引脚，表示通信结束
        HAL_GPIO_WritePin(spi_instances[i]->gpio_port, spi_instances[i]->gpio_pin, GPIO_PIN_SET);
        if (spi_instances[i]->spi_module_callback != NULL)      // 回调函数不为空就调用
        {
            spi_instances[i]->spi_module_callback(spi_instances[i]);
        }
        break; // 找到对应实例后退出循环
    }
    return;
}