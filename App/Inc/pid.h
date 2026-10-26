#ifndef PID_H
#define PID_H
typedef struct {
    float kp, ki, kd;
    float integral, previous_error;
    float minimum, maximum;
    unsigned initialized;
} Pid;
void Pid_Init(Pid *pid, float kp, float ki, float kd, float minimum, float maximum);
void Pid_Reset(Pid *pid);
float Pid_Update(Pid *pid, float target, float measured, float dt_seconds);
#endif
