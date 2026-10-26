#ifndef MOTOR_H
#define MOTOR_H
#include <stdint.h>
void Motor_Init(void);
/* Logical PWM excludes the configured dead-zone compensation. */
void Motor_SetPwm(unsigned id, int16_t logical_pwm);
void Motor_StopAll(uint8_t brake);
#endif
