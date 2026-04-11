#include "chassis.h"
#include "can.h"
#include "motor.h"
#include "pid.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

void chassis_init()
{
    ChassisInstance *instance = (ChassisInstance *)malloc(sizeof(ChassisInstance));
    memset(instance, 0, sizeof(ChassisInstance));
    
    PID_Init_Config_s pid_config = {
        .kp = 1,
        .ki = 0,
        .kd = 0,
        .i_max = 0,
        .out_max = 100,
    };
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
            .txconf = {
                .IDE = CAN_ID_STD,
                .RTR = CAN_RTR_DATA,
                .DLC = 0x08,
            },
            .rx_id = 0x201,
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
            .kp = 1,
            .ki = 0,
            .kd = 0,
            .i_max = 0,
            .out_max = 5000,
        },
        .ref = 0,
    };
    
    instance->motor_instance[0] = Motor_init_and_grouping(&motor_config, 1);
    instance->motor_instance[1] = Motor_init_and_grouping(&motor_config, 2);    
    instance->motor_instance[2] = Motor_init_and_grouping(&motor_config, 3);
    instance->motor_instance[3] = Motor_init_and_grouping(&motor_config, 4);
}
