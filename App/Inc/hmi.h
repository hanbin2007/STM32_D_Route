#ifndef HMI_H
#define HMI_H
#include <stdint.h>

void HMI_Init(void);
void HMI_Start(uint32_t now_ms);
void HMI_Stop(uint32_t now_ms);
/* call every App_Poll(); internally refreshes the OLED every 100 ms */
void HMI_Tick(uint32_t now_ms, const char *state, uint8_t fault,
              float dist_mm, float speed_mm_s);

#endif
