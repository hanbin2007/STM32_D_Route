#include "gray8.h"
#include "main.h"
#include "board_config.h"
#include <string.h>

void Gray8_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void wait_settle(void)
{
    const uint32_t cycles = (SystemCoreClock / 1000000U) * GRAY8_SETTLE_US;
    const uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) { __NOP(); }
}

void Gray8_ReadAll(Gray8Data *data)
{
    float sum = 0;
    memset(data, 0, sizeof(*data));
    for (unsigned channel=0; channel<8; ++channel) {
        HAL_GPIO_WritePin(GRAY_AD0_GPIO_Port, GRAY_AD0_Pin, (channel & 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRAY_AD1_GPIO_Port, GRAY_AD1_Pin, (channel & 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GRAY_AD2_GPIO_Port, GRAY_AD2_Pin, (channel & 4U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        wait_settle();
        data->raw[channel] = HAL_GPIO_ReadPin(GRAY_OUT_GPIO_Port, GRAY_OUT_Pin) == GPIO_PIN_SET ? 1U : 0U;
        if (data->raw[channel] == GRAY8_ACTIVE_LEVEL) {
            const unsigned index = GRAY8_X1_IS_LEFT ? channel : (7U-channel);
            data->active_mask |= (uint8_t)(1U << index);
            ++data->active_count;
            sum += (float)index - 3.5f;
        }
    }
    data->marker = data->active_count >= GRAY8_MARKER_MIN_ACTIVE ? 1U : 0U;
    data->line_valid = (data->active_count > 0U && !data->marker) ? 1U : 0U;
    if (data->line_valid) data->position = sum / (float)data->active_count;
}
