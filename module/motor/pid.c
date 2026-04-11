#include "pid.h"
#include <stdlib.h>
#include <string.h>

PIDInstance *PID_init(PID_Init_Config_s *config)
{
    PIDInstance *instance = (PIDInstance *)malloc(sizeof(PIDInstance));
    memset(instance, 0, sizeof(PIDInstance));

    instance->kd = config->kd;
    instance->ki = config->ki;
    instance->kp = config->kp;
    instance->i_max = config->i_max;
    instance->out_max = config->out_max;

    instance->err[0] = 0;
    instance->fdb = 0;
    instance->i_out = 0;
    instance->out = 0;
    instance->ref =0;

    return instance;
}

float PID_calc(PIDInstance *pid)
{
    pid->err[1] = pid->err[0];
    pid->err[0] = pid->ref - pid->fdb;

    pid->i_out += pid->ki * pid->err[0];
    if (pid->i_out > pid->i_max) pid->i_out = pid->i_max;
    if (pid->i_out < -pid->i_max) pid->i_out = -pid->i_max;

    pid->out = pid->kp * pid->err[0] + pid->i_out + pid->kd * (pid->err[0] - pid->err[1]);
    if (pid->out > pid->out_max) pid->out = pid->out_max;
    if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    
    return pid->out;
}