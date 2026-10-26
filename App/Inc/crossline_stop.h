#ifndef CROSSLINE_STOP_H
#define CROSSLINE_STOP_H
#include <stdint.h>
typedef struct { uint32_t clear_ms, marker_ms; uint8_t departed; } CrosslineStop;
void Crossline_Reset(CrosslineStop *stop);
uint8_t Crossline_Update(CrosslineStop *stop,uint8_t marker,uint8_t armed,uint32_t interval);
#endif
