#include "speed_control.h"
#include "pid.h"
#include "motor.h"
#include <math.h>
#include <string.h>
static Pid controllers[MOTOR_COUNT];
static float targets[MOTOR_COUNT], ramped[MOTOR_COUNT];
static int16_t pwm[MOTOR_COUNT];
static float clamp(float x, float lo, float hi) { return x<lo ? lo : (x>hi ? hi : x); }

void SpeedControl_Init(void)
{
    for (unsigned i=0; i<MOTOR_COUNT; ++i)
        Pid_Init(&controllers[i], SPEED_KP, SPEED_KI, SPEED_KD, 0,
                 (float)(MOTOR_PWM_LIMIT_COUNTS-MOTOR_DEADZONE_COUNTS));
    SpeedControl_Stop(0);
}
void SpeedControl_SetTargets(const float values[MOTOR_COUNT])
{
    for (unsigned i=0; i<MOTOR_COUNT; ++i)
        targets[i] = clamp(values[i], -SPEED_MAX_MM_S, SPEED_MAX_MM_S);
}
void SpeedControl_Tick(const float measured[MOTOR_COUNT], uint32_t interval_ms)
{
    const float dt = (float)interval_ms / 1000.0f;
    unsigned stopped=0;
    for (unsigned i=0; i<MOTOR_COUNT; ++i) {
        /* Ramp to zero before reversing, then accelerate in the new direction. */
        float next_target=ramped[i]*targets[i]<0 ? 0 : targets[i];
        float step = (fabsf(next_target)>fabsf(ramped[i]) ? SPEED_ACCEL_MM_S2 : SPEED_DECEL_MM_S2)*dt;
        float difference=next_target-ramped[i];
        ramped[i] += clamp(difference, -step, step);
        if (fabsf(ramped[i]) <= 0.5f && next_target == 0) {
            ramped[i]=0; pwm[i]=0; ++stopped;
            Pid_Reset(&controllers[i]);
            Motor_SetPwm(i, 0);
        } else {
            const float limit=(float)(MOTOR_PWM_LIMIT_COUNTS-MOTOR_DEADZONE_COUNTS);
            /* Overspeed may reduce effort to zero, but must not command the
             * opposite direction with the large motor dead-zone compensation. */
            controllers[i].minimum=ramped[i]<0 ? -limit : 0;
            controllers[i].maximum=ramped[i]<0 ? 0 : limit;
            pwm[i]=(int16_t)Pid_Update(&controllers[i], ramped[i], measured[i], dt);
            Motor_SetPwm(i, pwm[i]);
        }
    }
    if (stopped == MOTOR_COUNT) Motor_StopAll(1);
}
void SpeedControl_Stop(uint8_t brake)
{
    memset(targets,0,sizeof(targets)); memset(ramped,0,sizeof(ramped)); memset(pwm,0,sizeof(pwm));
    for (unsigned i=0; i<MOTOR_COUNT; ++i) Pid_Reset(&controllers[i]);
    Motor_StopAll(brake);
}
const float *SpeedControl_GetTargets(void) { return targets; }
const float *SpeedControl_GetRampedTargets(void) { return ramped; }
const int16_t *SpeedControl_GetPwm(void) { return pwm; }
