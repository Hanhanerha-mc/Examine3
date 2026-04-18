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

void chassisInit(void);
void chassisSetSpeed(float vx, float vy, float wz);
void chassis_set_rc_control(int16_t ch0, int16_t ch1, int16_t ch2, uint8_t sw2);

#endif
