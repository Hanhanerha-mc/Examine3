#include "pid.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

PIDInstance *PID_init(PID_Init_Config_s *config)
{
    PIDInstance *instance = (PIDInstance *)malloc(sizeof(PIDInstance));
    memset(instance, 0, sizeof(PIDInstance));

    instance->kd = config->kd;
    instance->ki = config->ki;
    instance->kp = config->kp;
    instance->i_max = config->i_max;
    instance->out_max = config->out_max;

    instance->deadBand = config->deadBand;
    instance->adaptiveKi = config->adaptiveKi;
    instance->dActivationThreshold = config->dActivationThreshold;

    instance->err_last = 0;
    instance->err = 0;
    instance->fdb = 0;
    instance->i_out = 0;
    instance->out = 0;
    instance->ref = 0;

    return instance;
}

float PID_calc(PIDInstance *pid)
{
    pid->err_last = pid->err;
    pid->err = pid->ref - pid->fdb;

    if (fabsf(pid->err) < pid->deadBand) {     //如果误差绝对值小于死区范围（死区未开启默认为0，恒不成立）
        pid->out = 0; // 在死区范围内输出为0
        return pid->out;
    }

    pid->i_out += pid->ki * pid->err;
    if (pid->i_out > pid->i_max) pid->i_out = pid->i_max;
    if (pid->i_out < -pid->i_max) pid->i_out = -pid->i_max;
    // 如果启动了自适应积分系数，根据误差和缩放因子计算自适应积分系数
    pid->adaptiveKi = (pid->iScalingFactor > 0) ? pid->err / pid->iScalingFactor : 1.0f;   

    // 根据误差范围决定是否启用微分作用
    pid->out = (fabsf(pid->err) > pid->dActivationThreshold) ? (pid->kp * pid->err + pid->i_out * pid->adaptiveKi) 
        : (pid->kp * pid->err + pid->i_out * pid->adaptiveKi + pid->kd * (pid->err - pid->err_last));

    if (pid->out > pid->out_max) pid->out = pid->out_max;
    if (pid->out < -pid->out_max) pid->out = -pid->out_max;
    
    return pid->out;
}