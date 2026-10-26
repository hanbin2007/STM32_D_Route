#ifndef OLED_H
#define OLED_H
#include <stdint.h>

/* Board J12 SSD1306 128x64: software I2C on PB6(SCL), PB7(SDA). */
void OLED_Init(void);
/* RAM drawing only; OLED_Service sends at most 16 data bytes per call. */
void OLED_Clear(void);
void OLED_ShowString(uint8_t x, uint8_t page, const char *text);
void OLED_Service(void);
/* 1 = board OLED acknowledged at address 0x3c, 0 = no display. */
uint8_t OLED_GetBus(void);
#endif
