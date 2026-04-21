#include "gimbal.h"
#include "motor.h"
#include <string.h>
#include "bsp.h"

GimbalInstance *gimbal = NULL;

void gimbalInit(void)
{
    Motor_Init_Config_s motorConfig = {
        .motor_type = GM6020,
        .can_config = {
            .can_handle = &hcan1,
            .rx_id = 0x201,
            // 回调函数与父指针在MotorInit中设置
        },
        .working_flag = RUNNING,
        .control_mode = ANGLE_CONTROL,      //云台电机，角度控制
        /*分组设置在电机分组函数中配置*/
        .speed_pid_config = {
            .kp = 3.5,
            .ki = 0,
            .kd = 0,
            .out_max = GM6050_CURRENT_MAX,

            .deadBand = 256,
        },
        .angle_pid_config = {
            .kp = 500,
            .ki = 0,
            .kd = 0,
            .i_max = 0,
            .out_max = 30000,
        },
        .ref = 0,
    };

    gimbal->motor_yaw = Motor_init_and_grouping(&motorConfig, 1);
    motorConfig.can_config.rx_id = 0x202; // 修改接收ID
    gimbal->motor_pitch = Motor_init_and_grouping(&motorConfig, 2); 

    return;
}

void gimbalAngle_yaw(float yaw_ref)
{
    if (gimbal == NULL) return;
    MotorSetAngle(gimbal->motor_yaw, yaw_ref);
}

void gimbalAngle_pitch(float pitch_ref)
{
    if (gimbal == NULL) return;
    MotorSetAngle(gimbal->motor_pitch, pitch_ref);
}
