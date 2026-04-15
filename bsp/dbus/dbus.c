#include "dbus.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_it.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>


/*进行Init即可，其他无需干涉，数据会存储到DBUS_Data结构体中*/


volatile DBUS_Data_s DBUS_Data = {0, 0, 0, 0, 0, 0, 0};
uint8_t DBUS_rxbuff[DBUS_BUFLEN];// 接收缓冲区

static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);

void DBUS_Init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART); // 清除IDLE标志位
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE); // 使能IDLE中断
    uart_receive_dma_no_it(&DBUS_HUART, DBUS_rxbuff, DBUS_BUFLEN); // 启动DMA接收
}

/**
* @brief 通过DMA直接内存访问高效存储数据（无需CPU参与）
*/
static int uart_receive_dma_no_it(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{
    if (huart->RxState == HAL_UART_STATE_READY) {       // 判断UART处于就绪状态
        if ((pData == NULL) || (Size == 0)) {      // 参数检查：确保数据指针有效且大小非零
            return HAL_ERROR;
        }
        
        huart->pRxBuffPtr = pData;      // 设置UART接收缓冲区指针，DMA将直接写入此地址
        huart->RxXferSize = Size;        // 设置UART接收数据大小，DMA将根据此值进行数据传输
        huart->ErrorCode = HAL_UART_ERROR_NONE; // 初始化错误码为无错误

        // 启动DMA接收，直接将数据存储到指定内存地址，无需CPU干预
        /*
        * @note 1. HAL库中，DMA接收通常通过HAL_UART_Receive_DMA函数启动，但此处直接调用底层函数以避免中断处理开销。
        *       2. 直接使用DMA进行数据传输可以显著提高效率  ，特别是在高数据速率或频繁接收的场景中。
        *       3. 启动DMA后，UART硬件将自动将接收的数据存储到指定的内存地址，直到达到设定的大小或发生错误。
        *       4. DMA接收完成后，会触发IDLE中断，需要在中断服务函数中处理。
        *       5. 注意：使用此函数时，确保DMA和UART的相关配置正确，以避免数据丢失或错误。
        * SrcAddress: UART数据寄存器地址，DMA将从此地址读取数据。
        * DstAddress: 用户提供的内存地址，DMA将直接将接收的数据存储到此地址。
        * DataLength: 要接收的数据大小，DMA将根据此值进行数据传输。
        */
        HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);
        SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);  // 使能UART DMA接收功能
        return HAL_OK;
    } else {
        return HAL_BUSY;
    }
}

static void DBUS_Callback(DBUS_Data_s *DBUS_Data, uint8_t *buff)
{
    // 解析DBUS_RX_BUF中的数据并存储到DBUS_Data结构体中
    DBUS_Data->ch0  = (int16_t)(((buff[0] | (buff[1] << 8))                        & 0x07FF) - RC_CH_VALUE_OFFSET); // 11位通道数据
    DBUS_Data->ch1  = (int16_t)(((buff[1] >> 3 | (buff[2] << 5))                   & 0x07FF) - RC_CH_VALUE_OFFSET); // 11位通道数据
    DBUS_Data->ch2  = (int16_t)(((buff[2] >> 6 | (buff[3] << 2) | (buff[4] << 10)) & 0x07FF) - RC_CH_VALUE_OFFSET); // 11位通道数据
    DBUS_Data->ch3  = (int16_t)(((buff[4] >> 1 | (buff[5] << 7))                   & 0x07FF) - RC_CH_VALUE_OFFSET); // 11位通道数据
    DBUS_Data->roll = (int16_t)(((buff[16] | (buff[17] << 8))                      & 0x07FF) - RC_CH_VALUE_OFFSET); // 滚转数据
    DBUS_Data->sw1  = ((buff[5] >> 4 & 0x000C)) >> 2;        // 开关1数据
    DBUS_Data->sw2  = (buff[5] >> 4) & 0x0003;               // 开关2数据
    
    if ((abs(DBUS_Data->ch0) > RC_CH_VALUE_MAX) || (abs(DBUS_Data->ch1) > RC_CH_VALUE_MAX) || 
        (abs(DBUS_Data->ch2) > RC_CH_VALUE_MAX) || (abs(DBUS_Data->ch3) > RC_CH_VALUE_MAX)) 
    {
        // 数据异常，可能是信号丢失或干扰，重置通道数据为0
        memset(DBUS_Data, 0, sizeof(DBUS_Data_s));
    }
}

uint16_t dma_current_data_counter(DMA_Stream_TypeDef *dma_stream)
{//返回DMA预定义的缓冲区剩余的长度，方便了解传输过程中还有多少数据尚未传输
  return ((uint16_t)(dma_stream->NDTR));
}

static void uart_rx_idle_callback(UART_HandleTypeDef* huart)
{
	__HAL_UART_CLEAR_IDLEFLAG(huart);
	//清除UART的空闲标志，以便下一次接收时能够正确检测到空闲状态
	
	if (huart == &DBUS_HUART)//确保只处理DBUS串口
	{
		__HAL_DMA_DISABLE(huart->hdmarx);//失能DMA接收，防止下一次接收的数据在上一次数据的尾部，而不是全新的数据

		if ((DBUS_MAX_LEN - dma_current_data_counter(huart->hdmarx->Instance)) == DBUS_BUFLEN)
		{//计算当前接收的数据长度，如果接收到的数据长度等于18字节，则调用处理数据函数
			DBUS_Callback(&DBUS_Data, DBUS_rxbuff);	//处理接收的数据并解码
		}
		__HAL_DMA_SET_COUNTER(huart->hdmarx, DBUS_MAX_LEN);//设置DMA接收预定义的缓冲区的长度，以便为下一次接收做好准备
		__HAL_DMA_ENABLE(huart->hdmarx);//重新启用DMA接收，以便继续接收数据
	}
}
void uart_receive_handler(UART_HandleTypeDef *huart)
{//用于检查UART接收状态并在接收到空闲状态时调用相应的回调函数
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && //检查UART是否设置了空闲标志，表示UART接收完成并进入空闲状态
			__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))//检查UART空闲中断是否被使能，只有在中断使能的情况下，才会处理空闲状态
	{
		uart_rx_idle_callback(huart);//调用之前定义的函数，处理接收到的数据
	}
}

