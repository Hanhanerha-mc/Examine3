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

    float ref;          // Reference 参考值
    float fdb;          // Feedback  反馈值
    float err[2];       // error     误差值
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
}PID_Init_Config_s;

PIDInstance *PID_init(PID_Init_Config_s *config);

void motor_PID_init(void);
float PID_calc(PIDInstance *pid);

#endif