#ifndef _BSP_CAN_H_
#define _BSP_CAN_H_

#include "main.h"
#include "can.h"

// 最多能够支持的CAN设备数
#define CAN_MX_REGISTER_CNT 16     // 这个数量取决于CAN总线的负载
#define MX_CAN_FILTER_CNT (2 * 14) // 最多可以使用的CAN过滤器数量,目前远不会用到这么多
#define DEVICE_CAN_CNT 2           // 根据板子设定,F407IG有CAN1,CAN2,因此为2;F334只有一个,则设为1
// 如果只有1个CAN,还需要把bsp_can.c中所有的hcan2变量改为hcan1(别担心,主要是总线和FIFO的负载均衡,不影响功能)

/* can instance typedef, every module registered to CAN should have this variable */
#pragma pack(1)
typedef struct _
{
    CAN_HandleTypeDef *can_handle; // can句柄
    CAN_TxHeaderTypeDef txconf;    // CAN报文发送配置
    uint32_t tx_id;                // 发送id
    uint32_t tx_mailbox;           // CAN消息填入的邮箱号
    uint8_t tx_buff[8];            // 发送缓存,发送消息长度可以通过CANSetDLC()设定,最大为8
    uint8_t rx_buff[8];            // 接收缓存,最大消息长度为8
    uint32_t rx_id;                // 接收id
    uint8_t rx_len;                // 接收长度,可能为0-8

    // 接收的回调函数,用于解析接收到的数据
    void (*can_module_callback)(struct _ *); // callback needs an instance to tell among registered ones
    void *parent_id;                                // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
} CANInstance;
#pragma pack()

/* CAN实例初始化结构体,将此结构体指针传入注册函数 */
typedef struct
{
    CAN_HandleTypeDef *can_handle;              // can句柄
    CAN_TxHeaderTypeDef txconf;                 // CAN报文发送配置
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(struct _ *);    // callback needs an instance to tell among registered ones
    void *parent_id;                            // 使用can外设的模块指针(即id指向的模块拥有此can实例,是父子关系)
}   CAN_Init_Config_s;

CANInstance *CANRegister(CAN_Init_Config_s *config);
uint8_t CANTransmit(CANInstance *instance, float timeout);
void CANSetDLC(CANInstance *_instance, uint8_t length);

#endif