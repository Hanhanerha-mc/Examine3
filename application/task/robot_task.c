#include "robot_task.h"
#include "bsp.h"
#include "chassis.h"

void StartMotorTask();

osThreadId motorTaskHandle;

void TaskInit()
{
    chassis_init(); // 初始化底盘电机

    osThreadDef(motorContrl, StartMotorTask, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motorContrl), NULL);
}

void StartMotorTask()
{
    while (1)
    {
        MotorContorl(); // 执行电机控制逻辑
        osDelay(10); // 10ms周期
    }
}