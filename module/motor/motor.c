#include "motor.h"
#include "bsp_can.h"
#include "can.h"
#include "pid.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

#define MAX_SPEED_REF 72000.0f
#define MAX_ANGLE_REF 360.0f

static uint8_t idx = 0;                                           // 电机实例索引
static MotorInstance *motorInstance[MAX_MOTOR_CNT] = {NULL};        // 电机实例指针数组
static uint8_t motorGroup[4] = {0, 0, 0, 0};      // 存储该组是否含有电机，0表示没有，1表示有

// 电机数据CAN实例列表初始化
static CANInstance can_motorSend_list[4] = {
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0} },
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x1FF, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0} },
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x1FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0} },
    [3] = {.can_handle = &hcan1, .txconf.StdId = 0x2FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .tx_buff = {0} },
};

static void MotorCallback(CANInstance *can_instance);             //Callback函数前置

/**
 * @brief 初始化电机实例
 * @param motor_config 电机初始化配置结构体指针
 * @return MotorInstance* 成功返回电机实例指针，失败返回NULL
 * @note 此函数负责创建电机实例，分配内存，初始化CAN通信，设置PID控制器，并配置工作状态
 */
MotorInstance *MotorInit(Motor_Init_Config_s *motor_config)       //初始化电机
{
    MotorInstance *motor_instance = (MotorInstance *)malloc(sizeof(MotorInstance));
    memset(motor_instance, 0, sizeof(MotorInstance));
    // 注：以下为CAN配置示例代码（已注释）
    // CAN_Init_Config_s can_config = {            //CAN实例初始化结构体
    //     .can_handle = &hcan1,
    //     .can_module_callback = MotorCallback,       //回调函数
    //     .parent_id = &motor_instance,                //父模块指针
    // };

    // 3. 初始化CAN实例
    // 3.1 注册CAN实例
    CANInstance *can_instance = CANRegister(&motor_config->can_config);
    // 3.2 设置CAN实例的父模块指针为当前电机实例
    can_instance->parent_id = motor_instance;
    // 3.3 设置CAN回调函数为MotorCallback
    can_instance->can_module_callback = MotorCallback;

    // 4. 初始化电机实例的基本属性
    motor_instance->motor_type = motor_config->motor_type;
    // 4.2 关联CAN实例
    motor_instance->motor_can_instance = can_instance;
    // 4.3 设置工作状态为运行中（默认启动状态）
    motor_instance->working_flag = RUNNING;
    // 4.4 设置控制模式为停止控制
    motor_instance->control_mode = STOP_CONTROL;
    
    // 5. 初始化PID控制器
    motor_instance->angle_pid = PID_init(&motor_config->angle_pid_config);
    motor_instance->speed_pid = PID_init(&motor_config->speed_pid_config);

    motorInstance[idx] = motor_instance;
    idx++;
    // 6. 返回初始化完成的电机实例
    return motor_instance;
}

MotorInstance *Motor_init_and_grouping(Motor_Init_Config_s *motor_config, uint8_t id)
{
    // 1. 初始化电机实例
    MotorInstance *motor_instance = MotorInit(motor_config);
    // if (motor_instance == NULL)
    // {
    //     // 处理初始化失败的情况
    //     return NULL;
    // }
    
    // 2. 根据电机类型进行分组
    switch (motor_config->motor_type)
    {
        case M2006:
        case M3508:
            if (id <= 4) {
                motorGroup[0] = 1; 
                motor_instance->sender_group = 0; // 分组0
                motor_instance->message_id = id;
            } else if (id <= 8) {
                motorGroup[1] = 1;
                motor_instance->sender_group = 1; // 分组1
                motor_instance->message_id = id - 4;
            }
            break;
        case GM6020:
            if (id <= 4) 
            {
                motorGroup[2] = 1; 
                motor_instance->sender_group = 2; // 分组2
                motor_instance->message_id = id;
            } else if (id <= 8) {
                motorGroup[3] = 1;
                motor_instance->sender_group = 3; // 分组3
                motor_instance->message_id = id - 4;
            }
            break;
        default:
            break;
    }
    return motor_instance;
}

void MotorControl(void)
{
    // 检索各个电机
    if (idx == 0) return; // 如果没有电机实例，直接返回
    for (uint8_t i = 0; i < idx; i++) {
        MotorInstance *instance = motorInstance[i];            // 获取电机实例指针
        Motor_Measure_s *measure = &(instance->measure);       // 获取电机测量数据指针 (需要取地址)
        float ref = instance->ref;                             // 获取电机控制参考值
        
        PIDInstance *speed_pid = instance->speed_pid;       //  直接保存一次指针引用从而减小访存的开销
        switch (instance->control_mode) {
            case ANGLE_CONTROL:
                PIDInstance *angle_pid = instance->angle_pid;       //  直接保存一次指针引用从而减小访存的开销
                // angle_pid->err[1] = angle_pid->err[0];
                // angle_pid->err[0] = measure->total_angle - ref; // 计算角度误差
                angle_pid->ref = ref;
                angle_pid->fdb = measure->total_angle;
                PID_calc(angle_pid);              // 计算PID输出，更新控制指令
                ref = angle_pid->out;   // 将角度PID的输出作为速度PID的反馈输入 
            case SPEED_CONTROL:
            case STOP_CONTROL:
                // speed_pid->err[1] = speed_pid->err[0];
                // speed_pid->err[0] = measure->speed_aps - ref; // 计算速度误差
                speed_pid->ref = ref;
                speed_pid->fdb = measure->speed_aps;
                PID_calc(speed_pid);              // 计算PID输出，更新控制指令
                break;
        }
        int16_t current_cmd = (int16_t)speed_pid->out; // DJI电机电流指令为有符号16位
        can_motorSend_list[instance->sender_group].tx_buff[(instance->message_id - 1) * 2] = (current_cmd >> 8) & 0xFF; // 将控制指令打包到CAN发送缓冲区
        can_motorSend_list[instance->sender_group].tx_buff[instance->message_id * 2 - 1] = current_cmd & 0xFF; // 将控制指令打包到CAN发送缓冲区
    }

    // 发送控制指令到电机
    for (uint8_t i = 0; i < 4; i++) {
        if (motorGroup[i]) CANTransmit(&can_motorSend_list[i], 10);  // 发送控制指令到电机，设置10ms超时时间
    }
}

// /**
// * @brief 电机数据传输函数
// * @param 
// * @return 无
// * @note 此函数负责向电机发送控制指令，如速度、角度、电流等控制信息
// * @details 函数执行流程：
// *          1. 遍历所有已初始化的电机实例
// *          2. 收集各电机的控制指令（如PID输出）
// *          3. 按照CAN通信协议打包数据
// *          4. 通过CAN总线发送控制指令到电机
// */
// void MotorTransmit(void)
// {
//     // TODO: 实现电机控制指令的发送逻辑
//     // 1. 遍历电机实例
//     for (int i = 0; i < 4; i++) {
//         if (motorGroup[i]) CANTransmit(can_instance[i]);
//     }
// }

void MotorWorkClose(MotorInstance *instance)
{
    instance->working_flag = STOP;
}

void MotorModeSwitch(MotorInstance *instance, Motor_control_mode_e mode)
{
    instance->working_flag = RUNNING;
    instance->control_mode = mode;
}

void MotorSetAngle(MotorInstance *instance, float ref)
{
    ref = clamp_float(ref, 0, MAX_ANGLE_REF);
    if (ref > 360.0f) ref -= 360.0f; // 将角度限制在[0, 360]范围内

    MotorModeSwitch(instance, ANGLE_CONTROL);      // 切换到角度控制模式

    // float delta = instance->measure.total_angle % 360.0f - ref;
    // if (delta > 180.0f) instance->ref = ref + (instance->measure.total_round / 36 + 1) * 360.0f;
    // else if (delta < -180.0f) instance->ref = ref + (instance->measure.total_round / 36 - 1) * 360.0f;
    // else instance->ref = ref + (instance->measure.total_round / 36) * 360.0f; // 角度控制的参考值需要考虑当前轮数,先整除36,再乘以360度,得到当前轮数

    ref = ref + (instance->measure.total_round / 36) * 360.0f; // 角度控制的参考值需要考虑当前轮数,先整除36,再乘以360度,得到当前轮数
    float delta = instance->measure.total_angle - ref;
    if (delta > 180.0f) ref += 360.0f;
    else if (delta < -180.0f) ref -= 360.0f;
    instance->ref = ref;
    
    // instance->ref = ref + (instance->measure.total_round / 36) * 360.0f; // 角度控制的参考值需要考虑当前轮数,先整除36,再乘以360度,得到当前轮数
    // /*目前用于考核的位置闭环控制，后续考虑与云台坐标系为参考*/
    instance->angle_pid->err_last = 0; // 重置PID误差
    instance->angle_pid->i_out = 0; // 重置积分项
}

void MotorSetSpeed(MotorInstance *instance, float speed_ref)
{
    MotorModeSwitch(instance, SPEED_CONTROL);      // 切换到速度控制模式
    instance->ref = clamp_float(speed_ref, -MAX_SPEED_REF, MAX_SPEED_REF);
    // instance->speed_pid->err_last = 0; // 重置PID误差
    // instance->speed_pid->i_out = 0; // 重置积分项
}

static void MotorCallback(CANInstance *can_instance)
{
    uint8_t *rx_buff = can_instance->rx_buff;
    MotorInstance *motor_instance = (MotorInstance *)can_instance->parent_id;
    Motor_Measure_s *measure = &motor_instance->measure;
    Motor_Type_e type = motor_instance->motor_type;

    measure->last_ecd = measure->ecd;
    measure->ecd = (uint16_t)rx_buff[0] << 8 | rx_buff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rx_buff[2] << 8 | rx_buff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rx_buff[4] << 8 | rx_buff[5]));
    measure->temperature = rx_buff[6];

    // 处理角度溢出，解卷绕
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    if (type == M2006) {
        measure->total_angle = measure->total_round * 10 + measure->angle_single_round / 36;
    }
    else if (type == M3508) {
        measure->total_angle = measure->total_round * 360 / 19 + measure->angle_single_round / 19;
    }
    else if (type == GM6020) {
        measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
    }
}