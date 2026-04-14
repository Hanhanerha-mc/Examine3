#ifndef _CHASSIS_H_
#define _CHASSIS_H_

#include "pid.h"
#include "motor.h"
#include <stdint.h>

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
ChassisInstance *chassis_get_instance(void);
void chassis_set_velocity(float vx, float vy, float wz);
void chassis_set_rc_control(int16_t ch0, int16_t ch1, int16_t ch2, uint8_t sw2);

#endif
