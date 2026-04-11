#include "robot_task.h"
#include "bsp.h"
#include "chassis.h"

void StartMotorTask(void const *argument);

osThreadId motorTaskHandle;

void TaskInit()
{
    chassis_init(); // 初始化底盘电机

    osThreadDef(motorContrl, StartMotorTask, osPriorityNormal, 0, 256);
    motorTaskHandle = osThreadCreate(osThread(motorContrl), NULL);
}

void StartMotorTask(void const *argument)
{
    while (1)
    {
        
        osDelay(10); // 10ms周期
    }
}