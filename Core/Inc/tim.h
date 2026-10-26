#ifndef TIM_H
#define TIM_H
#include "main.h"
extern TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim8;
void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM4_Init(void);
void MX_TIM5_Init(void);
void MX_TIM8_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#endif
