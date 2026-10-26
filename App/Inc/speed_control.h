#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H
#include <stdint.h>
#include "board_config.h"
void SpeedControl_Init(void);
void SpeedControl_SetTargets(const float targets[MOTOR_COUNT]);
void SpeedControl_Tick(const float measured[MOTOR_COUNT], uint32_t interval_ms);
void SpeedControl_Stop(uint8_t brake);
const float *SpeedControl_GetTargets(void);
const float *SpeedControl_GetRampedTargets(void);
const int16_t *SpeedControl_GetPwm(void);
#endif
