/* Timer configuration matches YB-DSF01 motor/encoder wiring. */
#include "tim.h"
TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim5, htim8;

static void pwm_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    TIM_OC_InitTypeDef oc = {0};
    TIM_BreakDeadTimeConfigTypeDef bdtr = {0};
    const uint32_t channels[] = {TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4};
    htim->Instance = instance;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 3599; /* 72 MHz / 3600 = 20 kHz. */
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(htim) != HAL_OK) Error_Handler();
    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    for (unsigned i = 0; i < 4; ++i)
        if (HAL_TIM_PWM_ConfigChannel(htim, &oc, channels[i]) != HAL_OK) Error_Handler();
    bdtr.OffStateRunMode = TIM_OSSR_DISABLE;
    bdtr.OffStateIDLEMode = TIM_OSSI_DISABLE;
    bdtr.LockLevel = TIM_LOCKLEVEL_OFF;
    bdtr.DeadTime = 0;
    bdtr.BreakState = TIM_BREAK_DISABLE;
    bdtr.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    bdtr.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(htim, &bdtr) != HAL_OK) Error_Handler();
    HAL_TIM_MspPostInit(htim);
}

static void encoder_init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    TIM_Encoder_InitTypeDef encoder = {0};
    htim->Instance = instance;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 65535;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 6;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 6;
    if (HAL_TIM_Encoder_Init(htim, &encoder) != HAL_OK) Error_Handler();
}

void MX_TIM1_Init(void) { pwm_init(&htim1, TIM1); }
void MX_TIM8_Init(void) { pwm_init(&htim8, TIM8); }
void MX_TIM2_Init(void) { encoder_init(&htim2, TIM2); }
void MX_TIM3_Init(void) { encoder_init(&htim3, TIM3); }
void MX_TIM4_Init(void) { encoder_init(&htim4, TIM4); }
void MX_TIM5_Init(void) { encoder_init(&htim5, TIM5); }

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
    else if (htim->Instance == TIM8) __HAL_RCC_TIM8_CLK_ENABLE();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    if (htim->Instance == TIM1) {
        __HAL_RCC_GPIOE_CLK_ENABLE();
        __HAL_AFIO_REMAP_TIM1_ENABLE();
        gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_13 | GPIO_PIN_14;
        HAL_GPIO_Init(GPIOE, &gpio);
    } else if (htim->Instance == TIM8) {
        __HAL_RCC_GPIOC_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOC, &gpio);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_NOPULL;
    if (htim->Instance == TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_AFIO_REMAP_TIM2_PARTIAL_1();
        gpio.Pin = GPIO_PIN_15;
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &gpio);
    } else if (htim->Instance == TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_AFIO_REMAP_TIM3_PARTIAL();
        gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        HAL_GPIO_Init(GPIOB, &gpio);
    } else if (htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        __HAL_AFIO_REMAP_TIM4_ENABLE();
        gpio.Pin = GPIO_PIN_12 | GPIO_PIN_13;
        HAL_GPIO_Init(GPIOD, &gpio);
    } else if (htim->Instance == TIM5) {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}
