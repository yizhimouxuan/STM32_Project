#ifndef PID_H
#define PID_H

typedef struct
{
    // 控制器增益参数
    float Kp;
    float Ki;
    float Kd;

    // 输出限制
    float max_out;
    float max_iout;

    // 设定值和反馈值
    float set;
    float fdb;

    // 控制器输出
    float out;
    float Pout;
    float Iout;
    float Dout;

    // 微分缓冲区和误差历史
    float Dbuf[3];
    float error[3];
} pid_t;

// 函数声明
void pid_setup(pid_t *ctrl, const float gains[3], float output_limit, float integral_limit);
float pid_compute(pid_t *ctrl, float feedback, float target);

#endif