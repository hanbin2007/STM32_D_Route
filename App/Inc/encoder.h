#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>
#include "board_config.h"
typedef struct {
    int32_t count[MOTOR_COUNT];
    int16_t delta[MOTOR_COUNT];
    float speed_mm_s[MOTOR_COUNT];
    float travel_mm;
} EncoderData;
void Encoder_Init(void);
void Encoder_Update(uint32_t interval_ms);
void Encoder_ResetTrip(void);
const EncoderData *Encoder_GetData(void);
#endif
