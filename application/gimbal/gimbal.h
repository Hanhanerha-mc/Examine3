#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "bsp.h"
#include "motor.h"

typedef struct
{
    MotorInstance *motor_pitch;
    MotorInstance *motor_yaw;
    int16_t pitch_ref;
    int16_t yaw_ref;
} GimbalInstance;

#endif
