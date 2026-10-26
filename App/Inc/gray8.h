#ifndef GRAY8_H
#define GRAY8_H
#include <stdint.h>
typedef struct {
    uint8_t raw[8];    /* Address order: X1..X8, unchanged raw 0/1. */
    uint8_t active_mask; /* Normalized physical left->right, bit0 is left. */
    uint8_t active_count;
    uint8_t marker;
    uint8_t line_valid;
    float position;   /* -3.5 left .. +3.5 right; meaningful if line_valid. */
} Gray8Data;
void Gray8_Init(void);
void Gray8_ReadAll(Gray8Data *data);
#endif
