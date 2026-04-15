#ifndef _DBUS_H_
#define _DBUS_H_

#include "main.h"
#include "usart.h"
#include <stdint.h>

#define DBUS_MAX_LEN 50
#define DBUS_BUFLEN 18
#define DBUS_HUART huart3
#define RC_CH_VALUE_OFFSET 1024
#define RC_CH_VALUE_MAX 660

typedef struct
{
    int16_t ch0; // 通道0
    int16_t ch1; // 通道1
    int16_t ch2; // 通道2
    int16_t ch3; // 通道3
    int16_t roll; // 滚转
    uint8_t sw1; // 开关1
    uint8_t sw2; // 开关2
} DBUS_Data_s;

extern volatile DBUS_Data_s DBUS_Data;// 接收缓冲区

void DBUS_Init();
void uart_receive_handler(UART_HandleTypeDef *huart);

#endif
