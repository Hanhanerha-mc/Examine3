#include "chassis.h"
#include "can.h"
#include "motor.h"
#include "pid.h"
#include "dbus.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define CHASSIS_RC_DEADBAND 40
#define CHASSIS_VX_MAX 15000.0f
#define CHASSIS_VY_MAX 15000.0f
#define CHASSIS_WZ_MAX 15000.0f
#define CHASSIS_WHEEL_SPEED_MAX 15000.0f

static ChassisInstance *chassis = NULL;

static void chassis_stop(void)
{
    if (chassis == NULL) return;
    MotorSetSpeed(chassis->motor_lf, 0.0f);
    MotorSetSpeed(chassis->motor_rf, 0.0f);
    MotorSetSpeed(chassis->motor_lb, 0.0f);
    MotorSetSpeed(chassis->motor_rb, 0.0f);
}

void chassis_init()
{
    if (chassis != NULL) return;          // 已经初始化过了，直接返回
    ChassisInstance *instance = (ChassisInstance *)malloc(sizeof(ChassisInstance));
    memset(instance, 0, sizeof(ChassisInstance));
    
    // instance->speed_pid = PID_init(&pid_config);
    
    // Motor_Init_Config_s motor_config = {
    //     .motor_type = M3508,
    //     .can_config = {
    //         .can_handle = &hcan1,
    //         .can_module_callback = M3508_calc,
    //         .tx_id = 
    //     },
    // };

    Motor_Init_Config_s motor_config = {
        .motor_type = M3508,
        .can_config = {
            .can_handle = &hcan1,
            // .txconf = {
            //     .IDE = CAN_ID_STD,
            //     .RTR = CAN_RTR_DATA,
            //     .DLC = 0x08,
            // },
            .rx_id = 0x203,
            // 回调函数与父指针在MotorInit中设置
        },
        .working_flag = RUNNING,
        .control_mode = SPEED_CONTROL,      //底盘电机，速度控制
        /*分组设置在电机分组函数中配置*/
        .angle_pid_config = {
            .kp = 0,
            .ki = 0,
            .kd = 0,
            .i_max = 0,
            .out_max = 0,
        },
        .speed_pid_config = {
            .kp = 3.5,
            .ki = 0.2,
            .kd = 0,
            .i_max = 1200,
            .out_max = M3508_CURRENT_MAX,

            .deadBand = 256,
        },
        .ref = 0,
    };
    
    /*顺序是lf，rf，lb，rb*/
    instance->motor_lf = Motor_init_and_grouping(&motor_config, 3);
    motor_config.can_config.rx_id = 0x204; // 修改接收ID
    // motor_config.speed_pid_config.kp = 5;
    instance->motor_rf = Motor_init_and_grouping(&motor_config, 4);
    motor_config.can_config.rx_id = 0x202; // 修改接收ID
    // motor_config.speed_pid_config.kp = 5.2;
    instance->motor_lb = Motor_init_and_grouping(&motor_config, 2);
    motor_config.can_config.rx_id = 0x201; // 修改接收ID
    // motor_config.speed_pid_config.kp = 5;
    instance->motor_rb = Motor_init_and_grouping(&motor_config, 1);

    chassis = instance;
}

ChassisInstance *chassis_get_instance(void)
{
    return chassis;
}

void chassisSetSpeed(float vx, float vy, float wz)
{
    if (chassis == NULL) return;

    float lf = vx + vy + wz;
    float rf = vx - vy - wz;
    float lb = vx - vy + wz;
    float rb = vx + vy - wz;

    // 如果有任何一个轮子的速度超过最大值，则按比例缩放所有轮子的速度
    float max_abs = fabsf(lf);
    if (fabsf(rf) > max_abs) max_abs = fabsf(rf);
    if (fabsf(lb) > max_abs) max_abs = fabsf(lb);
    if (fabsf(rb) > max_abs) max_abs = fabsf(rb);

    if (max_abs > CHASSIS_WHEEL_SPEED_MAX && max_abs > 0.0f) {
        float ratio = CHASSIS_WHEEL_SPEED_MAX / max_abs;
        lf *= ratio;
        rf *= ratio;
        lb *= ratio;
        rb *= ratio;
    }

    //设置电机速度
    MotorSetSpeed(chassis->motor_lf, lf);
    MotorSetSpeed(chassis->motor_rf, rf);
    MotorSetSpeed(chassis->motor_lb, lb);
    MotorSetSpeed(chassis->motor_rb, rb);
}

static float normalize_rc_channel(int16_t ch)
{
    if (abs(ch) < CHASSIS_RC_DEADBAND) {        // 如果数据在死区范围内，返回0
        return 0.0f;
    }

    return clamp_float((float)ch / RC_CH_VALUE_MAX, -1.0f, 1.0f);
}

void chassis_set_rc_control(int16_t ch0, int16_t ch1, int16_t ch2, uint8_t sw2)
{
    if (chassis == NULL) return;

    /*还没有测试*/
    if (sw2 == 1) {
        chassis_stop();
        return;
    }

    //获取各个方向的速度分量的比例并计算速度分量
    float vx = normalize_rc_channel(ch0) * CHASSIS_VX_MAX;
    float vy = normalize_rc_channel(ch1) * CHASSIS_VY_MAX;
    float wz = normalize_rc_channel(ch2) * CHASSIS_WZ_MAX;
    // 设置底盘速度
    chassisSetSpeed(vx, vy, wz);
}

