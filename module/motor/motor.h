#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "bsp_can.h"
#include "bsp.h"
#include "pid.h"
#include <stdint.h>
#include <sys/types.h>

/* 滤波系数设置为1的时候即关闭滤波 */
#define SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DJI 0.043945f // (360/8192),将编码器值转化为角度制
#define RPM_2_ANGLE_PER_SEC 6.0f       // 将RMP（每分钟圈数）转换为每秒转n度

#define MAX_MOTOR_CNT 8


/* 电机类型枚举 */
typedef enum
{
    MOTOR_TYPE_NONE = 0,
    GM6020,
    M3508,
    M2006,
} Motor_Type_e;

/* 工作状态枚举 */
typedef enum
{
    RUNNING = 0,
    STOP,
} Motor_Working_Type_e;

/* 控制模式枚举 */
typedef enum
{
    STOP_CONTROL = 0,       // 停止控制，电机不动
    SPEED_CONTROL,
    ANGLE_CONTROL,
} Motor_control_mode_e;


/* DJI电机CAN反馈信息*/
typedef struct
{
    uint16_t last_ecd;        // 上一次读取的编码器值
    uint16_t ecd;             // 0-8191,刻度总共有8192格
    float angle_single_round; // 单圈角度
    float speed_aps;          // 角速度,单位为:度/秒
    int16_t real_current;     // 实际电流
    uint8_t temperature;      // 温度 Celsius

    float total_angle;        // 总角度,注意方向
    int32_t total_round;      // 总圈数,注意方向
} Motor_Measure_s;

/**
 * @brief DJI intelligent motor typedef
 *
 */
typedef struct
{
    Motor_Measure_s measure;                  // 电机测量值
    Motor_Type_e motor_type;                  // 电机类型
    CANInstance *motor_can_instance;          // 电机CAN实例
    PIDInstance *angle_pid;
    PIDInstance *speed_pid;
    float ref;                                // pid计算参考值

    // 分组发送设置
    uint8_t sender_group;                     // 发送分组
    uint8_t message_id;                       // 消息id

    Motor_Working_Type_e working_flag;        // 启停标志
    Motor_control_mode_e control_mode;        // 控制模式,如速度控制,角度控制等
    // uint32_t feed_cnt;
    // float dt;
} MotorInstance;

typedef struct 
{
    Motor_Type_e motor_type;
    CAN_Init_Config_s can_config;
    Motor_Working_Type_e working_flag;        // 启停标志
    Motor_control_mode_e control_mode;

    PID_Init_Config_s angle_pid_config;
    PID_Init_Config_s speed_pid_config;
    float ref;                                // pid计算参考值
} Motor_Init_Config_s;

MotorInstance *MotorInit(Motor_Init_Config_s *motor_config);
MotorInstance *Motor_init_and_grouping(Motor_Init_Config_s *motor_config, uint8_t id);

void MotorContorl(void);

void MotorWorkClose(MotorInstance *instance);
void MotorModeSwitch(MotorInstance *instance, Motor_control_mode_e mode);
void MotorSetSpeed(MotorInstance *instance, float ref);

// typedef struct
// {

// };

#endif
