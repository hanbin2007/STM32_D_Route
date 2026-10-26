#include "pid.h"
static float clamp(float v, float lo, float hi) { return v<lo ? lo : (v>hi ? hi : v); }
void Pid_Init(Pid *pid, float kp, float ki, float kd, float minimum, float maximum)
{
    pid->kp=kp; pid->ki=ki; pid->kd=kd;
    pid->minimum=minimum; pid->maximum=maximum;
    Pid_Reset(pid);
}
void Pid_Reset(Pid *pid) { pid->integral=0; pid->previous_error=0; pid->initialized=0; }
float Pid_Update(Pid *pid, float target, float measured, float dt_seconds)
{
    float error = target - measured;
    float derivative = pid->initialized ? (error-pid->previous_error)/dt_seconds : 0;
    float candidate = pid->integral + error*dt_seconds;
    float output = pid->kp*error + pid->ki*candidate + pid->kd*derivative;
    if ((output >= pid->minimum && output <= pid->maximum) ||
        (output > pid->maximum && error < 0) || (output < pid->minimum && error > 0))
        pid->integral = candidate;
    pid->previous_error=error;
    pid->initialized=1;
    return clamp(output, pid->minimum, pid->maximum);
}
