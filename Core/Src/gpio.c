#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    HAL_GPIO_WritePin(GPIOF, GRAY_AD0_Pin | GRAY_AD1_Pin | GRAY_AD2_Pin, GPIO_PIN_RESET);
    gpio.Pin = GRAY_AD0_Pin | GRAY_AD1_Pin | GRAY_AD2_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOF, &gpio);

    gpio.Pin = GRAY_OUT_Pin | KEY1_Pin | KEY2_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL; /* KEY1/2 already have external pull-ups. */
    HAL_GPIO_Init(GPIOG, &gpio);

    HAL_GPIO_WritePin(GPIOG, BEEP_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOG, STATUS_LED_Pin, GPIO_PIN_SET);
    gpio.Pin = BEEP_Pin | STATUS_LED_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &gpio);
}
