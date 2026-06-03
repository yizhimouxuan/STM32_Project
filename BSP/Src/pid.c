#include "pid.h"

#define CLAMP_VALUE(val, limit)      \
    do {                             \
        if ((val) > (limit))         \
            (val) = (limit);         \
        else if ((val) < -(limit))   \
            (val) = -(limit);        \
    } while (0)

void pid_setup(pid_t *ctrl, const float gains[3], float output_limit, float integral_limit)
{
    if (!ctrl || !gains)
        return;
    
    ctrl->Kp = gains[0];
    ctrl->Ki = gains[1];
    ctrl->Kd = gains[2];
    ctrl->max_out = output_limit;
    ctrl->max_iout = integral_limit;
    
    ctrl->Dbuf[0] = 0.0f;
    ctrl->Dbuf[1] = 0.0f;
    ctrl->Dbuf[2] = 0.0f;
    
    ctrl->error[0] = 0.0f;
    ctrl->error[1] = 0.0f;
    ctrl->error[2] = 0.0f;
    
    ctrl->Pout = 0.0f;
    ctrl->Iout = 0.0f;
    ctrl->Dout = 0.0f;
    ctrl->out  = 0.0f;
}

float pid_compute(pid_t *ctrl, float feedback, float target)
{
    if (!ctrl)
        return 0.0f;
    
    // 更新误差历史
    ctrl->error[2] = ctrl->error[1];
    ctrl->error[1] = ctrl->error[0];
    ctrl->set = target;
    ctrl->fdb = feedback;
    ctrl->error[0] = target - feedback;
    
    // 计算比例项
    ctrl->Pout = ctrl->Kp * ctrl->error[0];
    
    // 计算积分项
    ctrl->Iout += ctrl->Ki * ctrl->error[0];
    
    // 更新微分缓冲区
    ctrl->Dbuf[2] = ctrl->Dbuf[1];
    ctrl->Dbuf[1] = ctrl->Dbuf[0];
    ctrl->Dbuf[0] = ctrl->error[0] - ctrl->error[1];
    
    // 计算微分项
    ctrl->Dout = ctrl->Kd * ctrl->Dbuf[0];
    
    // 限制积分项
    CLAMP_VALUE(ctrl->Iout, ctrl->max_iout);
    
    // 计算总输出
    ctrl->out = ctrl->Pout + ctrl->Iout + ctrl->Dout;
    CLAMP_VALUE(ctrl->out, ctrl->max_out);
    
    return ctrl->out;
}
