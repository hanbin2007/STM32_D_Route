#ifndef HOST_MAIN_H
#define HOST_MAIN_H
#include <stdint.h>
#define BEEP_GPIO_Port 0U
#define BEEP_Pin 12U
#define STATUS_LED_GPIO_Port 0U
#define STATUS_LED_Pin 14U
#define GPIO_PIN_RESET 0U
#define GPIO_PIN_SET 1U
uint32_t HAL_GetTick(void);
void HAL_GPIO_WritePin(unsigned port, uint16_t pin, unsigned state);
#endif
