#include "encoder.h"
#include "tim.h"
#include <math.h>
#include <string.h>
static EncoderData data;
static uint16_t previous[MOTOR_COUNT];
static TIM_HandleTypeDef *const timers[MOTOR_COUNT] = {&htim4, &htim2, &htim5, &htim3};
static const int8_t direction[MOTOR_COUNT] = ENCODER_DIRECTION_SIGNS;

void Encoder_Init(void)
{
    memset(&data, 0, sizeof(data));
    for (unsigned i=0; i<MOTOR_COUNT; ++i) {
        __HAL_TIM_SET_COUNTER(timers[i], 0);
        previous[i] = 0;
        if (HAL_TIM_Encoder_Start(timers[i], TIM_CHANNEL_ALL) != HAL_OK) Error_Handler();
    }
}

void Encoder_Update(uint32_t interval_ms)
{
    const float mm_per_count = (3.14159265359f * WHEEL_DIAMETER_MM) / WHEEL_COUNTS_PER_REV;
    float distance = 0;
    if (interval_ms == 0U) return;
    for (unsigned i=0; i<MOTOR_COUNT; ++i) {
        uint16_t current = (uint16_t)__HAL_TIM_GET_COUNTER(timers[i]);
        int16_t delta = (int16_t)(uint16_t)(current - previous[i]);
        previous[i] = current;
        delta = (int16_t)((int32_t)delta * direction[i]);
        data.delta[i] = delta;
        data.count[i] += delta;
        data.speed_mm_s[i] = (float)delta * mm_per_count * 1000.0f / (float)interval_ms;
        distance += fabsf((float)delta * mm_per_count);
    }
    data.travel_mm += distance / (float)MOTOR_COUNT;
}

void Encoder_ResetTrip(void)
{
    memset(&data, 0, sizeof(data));
    for (unsigned i=0; i<MOTOR_COUNT; ++i)
        previous[i] = (uint16_t)__HAL_TIM_GET_COUNTER(timers[i]);
}
const EncoderData *Encoder_GetData(void) { return &data; }
