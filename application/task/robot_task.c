#include "robot_task.h"
#include "bsp.h"
#include "dbus.h"
#include "motor.h"
#include "chassis.h"
#include "main.h"

void StartMotorTask();
void StartTestTask();
void MotorTestTask();

osThreadId motorTaskHandle;
osThreadId testTaskHandle;
osThreadId remoteTaskHandle;

volatile float g_test_speed_ref = 0.0f;
volatile float g_test_speed_fdb = 0.0f;
volatile float g_test_speed_err = 0.0f;
volatile float g_test_speed_out = 0.0f;

void TaskInit()
{
    DBUS_Init(); // 初始化遥控器数据接收
    chassis_init(); // 初始化底盘电机

    // osThreadDef(text, StartTestTask, osPriorityNormal, 0, 256);
    // testTaskHandle = osThreadCreate(osThread(text), NULL);
    osThreadDef(motorContrl, StartMotorTask, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motorContrl), NULL);
    // osThreadDef(motorTest, MotorTestTask, osPriorityNormal, 0, 256);
    // testTaskHandle = osThreadCreate(osThread(motorTest), NULL);
}

void StartMotorTask()
{
    while (1)
    {
        chassis_set_rc_control(DBUS_Data.ch0, DBUS_Data.ch1, DBUS_Data.ch2, DBUS_Data.sw2);
        MotorContorl(); // 执行电机控制逻辑
        osDelay(10); // 10ms周期
    }
}

void MotorTestTask()
{
    Motor_Init_Config_s motor_config0 = {
    .motor_type = M2006,
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
    .control_mode = SPEED_CONTROL,
    .speed_pid_config = {
        .kp = 5,
        .ki = 0,
        .kd = 0,
        .i_max = 0,
        .out_max = 1000,
        },
        .ref = 0,  
    };
    MotorInstance *motor = Motor_init_and_grouping(&motor_config0, 1);
    
    // 检查初始化是否成功
    if (motor == NULL) {
        while (1) {
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
            osDelay(100);  // 快速闪烁表示初始化失败
        }
    }
    
    if (motor->speed_pid == NULL) {
        while (1) {
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
            osDelay(200);  // 中速闪烁表示 PID 初始化失败
        }
    }
    
    MotorModeSwitch(motor, SPEED_CONTROL);      // 切换到速度控制模式
    MotorSetSpeed(motor, 100); // 更新电机控制参考值

    while (1)
    {
        g_test_speed_ref = motor->speed_pid->ref;
        g_test_speed_fdb = motor->speed_pid->fdb;
        g_test_speed_err = motor->speed_pid->err;
        g_test_speed_out = motor->speed_pid->out;
        osDelay(25); // 25ms周期
    }
}

void StartTestTask()
{
    while (1)
    {
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin); // 切换PA5引脚状态
        osDelay(1000); // 1s周期
    }
}



// // 问题：lineKp 定义为 int16_t，但计算时可能溢出
// static int16_t lineKp = 24;  // 最大32767
// speedDelta = (int16_t)((lineKp * currentLateralError) / 2);
// // 当 lineKp=32767, currentLateralError=127 时，中间结果可能溢出

