#ifndef IMU_BUS_H
#define IMU_BUS_H
#include <stdint.h>
void ImuBus_Init(void);
void ImuBus_BootDelay(uint32_t milliseconds);
uint8_t ImuBus_Write(uint8_t address,uint8_t reg,uint8_t value);
uint8_t ImuBus_Read(uint8_t address,uint8_t reg,uint8_t *data,unsigned count);
#endif
