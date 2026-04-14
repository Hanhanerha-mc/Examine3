#include "chassis.h"
#include "can.h"
#include "motor.h"
#include "pid.h"
#include "dbus.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

#define CHASSIS_RC_DEADBAND 30
#define CHASSIS_VX_MAX 120.0f
#define CHASSIS_VY_MAX 120.0f
#define CHASSIS_WZ_MAX 80.0f
#define CHASSIS_WHEEL_SPEED_MAX 100.0f

static ChassisInstance *g_chassis = NULL;

static float normalize_rc_channel(int16_t ch)
{
    if (abs(ch) < CHASSIS_RC_DEADBAND) {
        return 0.0f;
    }

    return clamp_float((float)ch / RC_CH_VALUE_MAX, -1.0f, 1.0f);
}

static void chassis_stop(void)
{
    if (g_chassis == NULL) return;
    MotorSetSpeed(g_chassis->motor_lf, 0.0f);
    MotorSetSpeed(g_chassis->motor_rf, 0.0f);
    MotorSetSpeed(g_chassis->motor_lb, 0.0f);
    MotorSetSpeed(g_chassis->motor_rb, 0.0f);
}

void chassis_init()
{
    if (g_chassis != NULL) return;          // 已经初始化过了，直接返回
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
            .kp = 7.5,
            .ki = 0,
            .kd = 0,
            .i_max = 0,
            .out_max = 2000,
        },
        .ref = 0,
    };
    
    /*顺序是lf，rf，lb，rb*/
    instance->motor_lf = Motor_init_and_grouping(&motor_config, 3);
    motor_config.can_config.rx_id = 0x204; // 修改接收ID
    motor_config.speed_pid_config.kp = 5;
    instance->motor_rf = Motor_init_and_grouping(&motor_config, 4);
    motor_config.can_config.rx_id = 0x202; // 修改接收ID
    motor_config.speed_pid_config.kp = 6;
    instance->motor_lb = Motor_init_and_grouping(&motor_config, 2);
    motor_config.can_config.rx_id = 0x201; // 修改接收ID
    motor_config.speed_pid_config.kp = 8.5;
    instance->motor_rb = Motor_init_and_grouping(&motor_config, 1);

    g_chassis = instance;
}

ChassisInstance *chassis_get_instance(void)
{
    return g_chassis;
}

void chassis_set_velocity(float vx, float vy, float wz)
{
    if (g_chassis == NULL) return;

    float lf = vx + vy + wz;
    float rf = vx - vy - wz;
    float lb = vx - vy + wz;
    float rb = vx + vy - wz;

    // 速度归一化处理
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

    MotorSetSpeed(g_chassis->motor_lf, lf);
    MotorSetSpeed(g_chassis->motor_rf, rf);
    MotorSetSpeed(g_chassis->motor_lb, lb);
    MotorSetSpeed(g_chassis->motor_rb, rb);
}

void chassis_set_rc_control(int16_t ch0, int16_t ch1, int16_t ch2, uint8_t sw2)
{
    if (g_chassis == NULL) return;

    if (sw2 == 1) {
        chassis_stop();
        return;
    }

    float vx = normalize_rc_channel(ch1) * CHASSIS_VX_MAX;
    float vy = normalize_rc_channel(ch0) * CHASSIS_VY_MAX;
    float wz = normalize_rc_channel(ch2) * CHASSIS_WZ_MAX;
    chassis_set_velocity(vx, vy, wz);
}

