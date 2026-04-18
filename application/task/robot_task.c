#include "robot_task.h"
#include "bsp.h"
#include "dbus.h"
#include "motor.h"
#include "chassis.h"
#include "main.h"

void StartMotorTask();
// void StartTestTask();
// void MotorTestTask();
// void MotorAngleTask();
void StartChassisTask();

MotorInstance *motor; // 全局电机实例指针，供测试任务使用

volatile float g_test_speed_ref = 0.0f;
volatile float g_test_speed_fdb = 0.0f;
volatile float g_test_speed_err = 0.0f;
volatile float g_test_speed_out = 0.0f;
volatile float g_test_angle_ref = 0.0f;
volatile float g_test_angle_fdb = 0.0f;
volatile float g_test_angle_err = 0.0f;
volatile float g_test_angle_out = 0.0f;
volatile float test1 = 0.0f;
volatile float test2 = 0.0f;
volatile float test3 = 0.0f;
volatile float test4 = 0.0f;
volatile float test5 = 0.0f;

osThreadId_t chassisTaskHandle;
const osThreadAttr_t chassisTask_attributes = {
  .name = "chassisTask",
  .stack_size = 512 * sizeof(StackType_t),
  .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 256 * sizeof(StackType_t),
  .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t testTaskHandle;
const osThreadAttr_t testTask_attributes = {
  .name = "testTask",
  .stack_size = 256 * sizeof(StackType_t),
  .priority = (osPriority_t)osPriorityNormal,
};

void TaskInit()
{
    DBUS_Init(); // 初始化遥控器数据接收
    chassisInit(); // 初始化底盘电机

    chassisTaskHandle = osThreadNew(StartChassisTask, NULL, &chassisTask_attributes);
    motorTaskHandle = osThreadNew(StartMotorTask, NULL, &motorTask_attributes);
    // testTaskHandle = osThreadNew(MotorTestTask, NULL, &testTask_attributes);



    // osThreadDef(text, StartTestTask, osPriorityNormal, 0, 256);
    // testTaskHandle = osThreadCreate(osThread(text), NULL);
    // osThreadDef(motorContrl, StartMotorTask, osPriorityNormal, 0, 512);
    // motorTaskHandle = osThreadCreate(osThread(motorContrl), NULL);

    // osThreadDef(motorTest, MotorTestTask, osPriorityNormal, 0, 512);
    // testTaskHandle = osThreadCreate(osThread(motorTest), NULL);
    // osThreadDef(angleTest, MotorAngleTask, osPriorityNormal, 0, 256);
    // testTaskHandle = osThreadCreate(osThread(angleTest), NULL);
}

void StartChassisTask()
{
    
    while (1)
    {
        chassis_set_rc_control(DBUS_Data.ch0, DBUS_Data.ch1, DBUS_Data.ch2, DBUS_Data.sw2);
        MotorControl(); // 执行电机控制逻辑
        osDelay(2); // 2ms周期
    }
}

void MotorTestTask()
{
    // Motor_Init_Config_s motor_config0 = {
    //     .motor_type = M2006,
    //     .can_config = {
    //         .can_handle = &hcan1,
    //         .rx_id = 0x201,
    //             // 回调函数与父指针在MotorInit中设置
    //     },
    //     .working_flag = RUNNING,
    //     .control_mode = SPEED_CONTROL,
    //     .speed_pid_config = {
    //         .kp = 1.6,
    //         .ki = 0,
    //         .kd = 0,
    //         .i_max = 0,
    //         .out_max = 3000,
    //         },
    //         .ref = 0,  
    //     .angle_pid_config = {
    //         .kp = 700,
    //         .ki = 0,
    //         .kd = 10,
    //         .i_max = 0,
    //         .out_max = 30000,
    //     },
    // };
    // motor = Motor_init_and_grouping(&motor_config0, 1);
    
    // // 检查初始化是否成功
    // if (motor == NULL) {
    //     while (1) {
    //         HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    //         osDelay(100);  // 快速闪烁表示初始化失败
    //     }
    // }
    
    // if (motor->speed_pid == NULL) {
    //     while (1) {
    //         HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
    //         osDelay(200);  // 中速闪烁表示 PID 初始化失败
    //     }
    // }

    // MotorModeSwitch(motor, SPEED_CONTROL);      // 切换到角度控制模式
    // MotorSetSpeed(motor, 0);
    

    while (1)
    {
        MotorSetSpeed(motor, DBUS_Data.ch2 * 80); // 更新电机控制参考值
        MotorControl(); // 执行电机控制逻辑
        g_test_speed_ref = motor->speed_pid->ref;
        g_test_speed_fdb = motor->speed_pid->fdb;
        g_test_speed_err = motor->speed_pid->err;
        g_test_speed_out = motor->speed_pid->out;
        g_test_angle_ref = motor->angle_pid->ref;
        g_test_angle_fdb = motor->angle_pid->fdb;
        g_test_angle_err = motor->angle_pid->err;
        g_test_angle_out = motor->angle_pid->out;
        test1 = motor->measure.speed_aps;
        test2 = DBUS_Data.ch0;
        test3 = DBUS_Data.ch1;
        test4 = DBUS_Data.ch2;
        test5 = DBUS_Data.sw2;
        osDelay(5); // 5ms周期
    }
}

void MotorAngleTask()
{
    while (1)
    {
        MotorSetAngle(motor, 90.0f); // 设置目标角度为90度
        osDelay(1000); // 每1秒更新一次目标角度
        MotorSetAngle(motor, 180.0f); // 设置目标角度为90度
        osDelay(1000); // 每1秒更新一次目标角度
        MotorSetAngle(motor, 270.0f); // 设置目标角度为90度
        osDelay(1000); // 每1秒更新一次目标角度
        MotorSetAngle(motor, 60.0f); // 设置目标角度为90度
        osDelay(1000); // 每1秒更新一次目标角度
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

