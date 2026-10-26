/* USER CODE BEGIN Header */
/* STM32F103ZET6 / Yahboom YB-DSF01. Board pin names follow the schematic. */
/* USER CODE END Header */
#ifndef MAIN_H
#define MAIN_H
#include "stm32f1xx_hal.h"

void Error_Handler(void);
#define GRAY_AD2_Pin GPIO_PIN_13
#define GRAY_AD2_GPIO_Port GPIOF
#define GRAY_AD1_Pin GPIO_PIN_14
#define GRAY_AD1_GPIO_Port GPIOF
#define GRAY_AD0_Pin GPIO_PIN_15
#define GRAY_AD0_GPIO_Port GPIOF
#define GRAY_OUT_Pin GPIO_PIN_0
#define GRAY_OUT_GPIO_Port GPIOG
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOG
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOG
#define BEEP_Pin GPIO_PIN_12
#define BEEP_GPIO_Port GPIOG
#define STATUS_LED_Pin GPIO_PIN_14
#define STATUS_LED_GPIO_Port GPIOG
#define HAL_1A_Pin GPIO_PIN_12
#define HAL_1A_GPIO_Port GPIOD
#define HAL_1B_Pin GPIO_PIN_13
#define HAL_1B_GPIO_Port GPIOD
#define HAL_2A_Pin GPIO_PIN_15
#define HAL_2A_GPIO_Port GPIOA
#define HAL_2B_Pin GPIO_PIN_3
#define HAL_2B_GPIO_Port GPIOB
#define HAL_3A_Pin GPIO_PIN_0
#define HAL_3A_GPIO_Port GPIOA
#define HAL_3B_Pin GPIO_PIN_1
#define HAL_3B_GPIO_Port GPIOA
#define HAL_4A_Pin GPIO_PIN_4
#define HAL_4A_GPIO_Port GPIOB
#define HAL_4B_Pin GPIO_PIN_5
#define HAL_4B_GPIO_Port GPIOB
#define Motor1_A_Pin GPIO_PIN_6
#define Motor1_B_Pin GPIO_PIN_7
#define Motor2_A_Pin GPIO_PIN_8
#define Motor2_B_Pin GPIO_PIN_9
#define Motor3_A_Pin GPIO_PIN_9
#define Motor3_B_Pin GPIO_PIN_11
#define Motor4_A_Pin GPIO_PIN_13
#define Motor4_B_Pin GPIO_PIN_14
#endif
