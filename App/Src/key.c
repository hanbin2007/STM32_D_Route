#include "key.h"
#include "main.h"
#include "board_config.h"
static uint8_t previous_raw, stable;
static uint32_t changed_at;
void Key_Init(uint32_t now_ms)
{
    previous_raw=stable=HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET ? 1U : 0U;
    changed_at=now_ms;
}
uint8_t Key_StartPressed(uint32_t now_ms)
{
    uint8_t raw=HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET ? 1U : 0U;
    if (raw != previous_raw) { previous_raw=raw; changed_at=now_ms; }
    if (raw != stable && (uint32_t)(now_ms-changed_at) >= KEY_DEBOUNCE_MS) {
        stable=raw;
        return stable;
    }
    return 0;
}
uint8_t Key_StopHeld(void)
{
    return HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET ? 1U : 0U;
}
