#include "bsp_can.h"
#include "dwt.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL}; //can_instance指针数组,存储实例，用于回调时索引不同实例
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增
// static uint8_t can_service_initialized = 0; // CAN服务启动标志

static void CANAddFilter(CANInstance *_instance);
void CAN_ServiceInit();

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    if (idx >= CAN_MX_REGISTER_CNT)     return NULL; //超过最大实例的操作需补充
    if (!idx)                           CAN_ServiceInit();
    /*重复注册逻辑需补充*/

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));
    // 进行发送报文的配置
    instance->txconf.StdId = config->tx_id; // 发送id
    instance->txconf.IDE = CAN_ID_STD;      // 使用标准id,扩展id则使用CAN_ID_EXT(目前没有需求)
    instance->txconf.RTR = CAN_RTR_DATA;    // 发送数据帧
    instance->txconf.DLC = 0x08;            // 默认发送长度为8
    // 设置回调函数和接收发送id
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id; // 好像没用,可以删掉
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->parent_id = config->parent_id;

    CANAddFilter(instance);         // 添加CAN过滤器规则
    can_instance[idx++] = instance; // 将实例保存到can_instance中
    return instance; // 返回can实例指针
}

/**
 * @brief 为CAN实例添加过滤器配置
 * @param _instance CAN实例指针
 * @return 无
 * @note 此函数负责为每个CAN实例配置独立的过滤器，确保只接收与该实例相关的CAN报文
 * @details 函数执行流程：
 *          1. 初始化过滤器配置结构体
 *          2. 根据CAN实例的属性配置过滤器参数
 *          3. 调用HAL库函数应用过滤器配置
 */
static void CANAddFilter(CANInstance *_instance)
{
    // 1. 定义CAN过滤器配置结构体
    CAN_FilterTypeDef can_filter_conf = {0};
    
    // 2. 静态变量，用于跟踪过滤器索引  
    //    0-13号过滤器给CAN1使用，14-27号过滤器给CAN2使用
    static uint8_t can1_filter_idx = 0, can2_filter_idx = 14; 

    // 3. 配置过滤器模式为ID列表模式
    //    只有将rxid添加到过滤器中才会接收到，其他报文会被过滤
    can_filter_conf.FilterMode = CAN_FILTERMODE_IDLIST;
    
    // 4. 配置过滤器缩放为16位ID模式
    //    只有低16位有效
    can_filter_conf.FilterScale = CAN_FILTERSCALE_16BIT;
    
    // 5. 根据RX ID的奇偶性分配FIFO
    //    奇数ID的模块会被分配到FIFO0，偶数ID的模块会被分配到FIFO1
    can_filter_conf.FilterFIFOAssignment = (_instance->rx_id & 1) ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    
    // 6. 设置从机过滤器起始银行
    //    在STM32的BxCAN控制器中，CAN2是CAN1的从机，从第14个过滤器开始
    can_filter_conf.SlaveStartFilterBank = 14;
    
    // 7. 16-bit IDLIST 模式下，同一个 bank 可以放 4 个标准ID。
    //    这里将4个槽位都写入同一个 rx_id，避免未初始化槽位造成误匹配/漏匹配。
    uint16_t std_id_16b = (uint16_t)(_instance->rx_id << 5);
    can_filter_conf.FilterIdLow = std_id_16b;
    can_filter_conf.FilterIdHigh = std_id_16b;
    can_filter_conf.FilterMaskIdLow = std_id_16b;
    can_filter_conf.FilterMaskIdHigh = std_id_16b;

    // 8. 根据CAN实例的CAN句柄分配过滤器银行
    //    如果是CAN1，则使用can1_filter_idx并自增
    //    如果是CAN2，则使用can2_filter_idx并自增
    can_filter_conf.FilterBank = _instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
    
    // 9. 启用过滤器
    can_filter_conf.FilterActivation = CAN_FILTER_ENABLE;

    // 10. 调用HAL库函数应用过滤器配置
    if (HAL_CAN_ConfigFilter(_instance->can_handle, &can_filter_conf) != HAL_OK)
    {
        Error_Handler();
    }
}

void CAN_ServiceInit()
{
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
    {
        Error_Handler();
    }
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    // static uint32_t busy_count;         //记录超时次数
    // static volatile float wait_time __attribute__((unused)); // for cancel warning
    // float dwt_start = DWT_GetTimeline_ms();
    // while (HAL_CAN_GetTxMailboxesFreeLevel(_instance->can_handle) == 0) // 等待邮箱空闲
    // {
    //     if (DWT_GetTimeline_ms() - dwt_start > timeout) // 超时
    //     {
    //         // LOGWARNING("[bsp_can] CAN MAILbox full! failed to add msg to mailbox. Cnt [%d]", busy_count);
    //         busy_count++;
    //         return 1;
    //     }
    // }
    // wait_time = DWT_GetTimeline_ms() - dwt_start;
    // // tx_mailbox会保存实际填入了这一帧消息的邮箱,但是知道是哪个邮箱发的似乎也没啥用
    // if (HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox))
    // {
    //     // LOGWARNING("[bsp_can] CAN bus BUS! cnt:%d", busy_count);
    //     busy_count++;
    //     return 1;
    // }
    HAL_CAN_AddTxMessage(_instance->can_handle, &_instance->txconf, _instance->tx_buff, &_instance->tx_mailbox);
    return 0;       //return 0 为发送成功
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    // 发送长度错误!检查调用参数是否出错,或出现野指针/越界访问
    if (length > 8 || length == 0) // 安全检查，暂时直接return处理
        // while (1)
        //     LOGERROR("[bsp_can] CAN DLC error! check your code or wild pointer");
        return;
    _instance->txconf.DLC = length;
}

/**
 * @brief 此函数会被下面两个函数调用,用于处理FIFO0和FIFO1溢出中断(说明收到了新的数据)
 *        所有的实例都会被遍历,找到can_handle和rx_id相等的实例时,调用该实例的回调函数
 *
 * @param _hcan
 * @param fifox passed to HAL_CAN_GetRxMessage() to get mesg from a specific fifo
 */
static void CANFIFOxCallback(CAN_HandleTypeDef *_hcan, uint32_t fifox)
{
    static CAN_RxHeaderTypeDef rxconf; // 同上
    uint8_t can_rx_buff[8];
    while (HAL_CAN_GetRxFifoFillLevel(_hcan, fifox)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_CAN_GetRxMessage(_hcan, fifox, &rxconf, can_rx_buff); // 从FIFO中获取数据
        for (size_t i = 0; i < idx; ++i)
        { // 两者相等说明这是要找的实例
            if (_hcan == can_instance[i]->can_handle && rxconf.StdId == can_instance[i]->rx_id)
            {
                if (can_instance[i]->can_module_callback != NULL)      // 回调函数不为空就调用
                {
                    can_instance[i]->rx_len = rxconf.DLC;                      // 保存接收到的数据长度
                    memcpy(can_instance[i]->rx_buff, can_rx_buff, rxconf.DLC); // 消息拷贝到对应实例
                    can_instance[i]->can_module_callback(can_instance[i]);     // 触发回调进行数据解析和处理，根据不同module的模块进行处理
                }
                return;
            }
        }
    }
}

/**
 * @brief CAN回调处理函数，调用上方函数进行数据处理
 * @note STM32的两个CAN设备共享两个FIFO
 *       当FIFO0或FIFO1溢出时会调用这两个函数
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO0);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CANFIFOxCallback(hcan, CAN_RX_FIFO1);
}