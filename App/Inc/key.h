#ifndef KEY_H
#define KEY_H
#include <stdint.h>
void Key_Init(uint32_t now_ms);
uint8_t Key_StartPressed(uint32_t now_ms);
uint8_t Key_StopHeld(void);
#endif
