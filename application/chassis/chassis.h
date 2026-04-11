#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "pid.h"
#include "motor.h"

typedef struct
{
    MotorInstance *motor_lf;
    MotorInstance *motor_rf;
    MotorInstance *motor_lb;
    MotorInstance *motor_rb;
    PIDInstance *speed_pid;
} ChassisInstance;

typedef struct
{
    MotorInstance *motor_instance[4];
    PIDInstance *speed_pid;
} Chassis_Init_Config_s; 

void chassis_init();

#endif
