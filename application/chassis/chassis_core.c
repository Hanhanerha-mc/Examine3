// #include "chassis.h"

// void chassisInit(void)
// {
//     if (chassis != NULL) return;          // 已经初始化过了，直接返回
//     ChassisInstance *instance = (ChassisInstance *)malloc(sizeof(ChassisInstance));
//     memset(instance, 0, sizeof(ChassisInstance));
    
//     // instance->speed_pid = PID_init(&pid_config);
    
//     // Motor_Init_Config_s motor_config = {
//     //     .motor_type = M3508,
//     //     .can_config = {
//     //         .can_handle = &hcan1,
//     //         .can_module_callback = M3508_calc,
//     //         .tx_id = 
//     //     },
//     // };

//     Motor_Init_Config_s motor_config = {
//         .motor_type = M3508,
//         .can_config = {
//             .can_handle = &hcan1,
//             // .txconf = {
//             //     .IDE = CAN_ID_STD,
//             //     .RTR = CAN_RTR_DATA,
//             //     .DLC = 0x08,
//             // },
//             .rx_id = 0x203,
//             // 回调函数与父指针在MotorInit中设置
//         },
//         .working_flag = RUNNING,
//         .control_mode = SPEED_CONTROL,      //底盘电机，速度控制
//         /*分组设置在电机分组函数中配置*/
//         .angle_pid_config = {
//             .kp = 0,
//             .ki = 0,
//             .kd = 0,
//             .i_max = 0,
//             .out_max = 0,
//         },
//         .speed_pid_config = {
//             .kp = 3.5,
//             .ki = 0.2,
//             .kd = 0,
//             .i_max = 1200,
//             .out_max = M3508_CURRENT_MAX,

//             .deadBand = 256,
//         },
//         .ref = 0,
//     };
    
//     /*顺序是lf，rf，lb，rb*/
//     instance->motor_lf = Motor_init_and_grouping(&motor_config, 3);
//     motor_config.can_config.rx_id = 0x204; // 修改接收ID
//     // motor_config.speed_pid_config.kp = 5;
//     instance->motor_rf = Motor_init_and_grouping(&motor_config, 4);
//     motor_config.can_config.rx_id = 0x202; // 修改接收ID
//     // motor_config.speed_pid_config.kp = 5.2;
//     instance->motor_lb = Motor_init_and_grouping(&motor_config, 2);
//     motor_config.can_config.rx_id = 0x201; // 修改接收ID
//     // motor_config.speed_pid_config.kp = 5;
//     instance->motor_rb = Motor_init_and_grouping(&motor_config, 1);

//     chassis = instance;
// }
