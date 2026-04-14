#ifndef _PID_H_
#define _PID_H_

/**
 * @brief PID变量声明
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_out;
    float i_max;
    float out_max;

    float deadBand;     // 死区范围，绝对值小于该值时输出为0
    float adaptiveKi;   // 自适应积分系数
    float iScalingFactor; // 缩放因子，用于调整积分值的范围
    float dActivationThreshold; // 微分作用阈值，当误差处于这一范围内时微分作用才会生效
    
    float ref;          // Reference 参考值
    float fdb;          // Feedback  反馈值
    float err_last;     // LastError 上一次误差值
    float err;          // Error     当前误差值
    float out;          // out       输出值

}PIDInstance;

/**
 * @brief PID变量声明
 */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_max;
    float out_max;
    float deadBand;     // 死区范围
    float adaptiveKi;   // 自适应积分系数
    float dActivationThreshold; // 微分作用阈值，当误差处于这一范围内时微分作用才会生效
}PID_Init_Config_s;

PIDInstance *PID_init(PID_Init_Config_s *config);

void motor_PID_init(void);
float PID_calc(PIDInstance *pid);

#endif