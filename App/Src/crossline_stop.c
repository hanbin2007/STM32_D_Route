#include "crossline_stop.h"
#include "board_config.h"
void Crossline_Reset(CrosslineStop *s) { s->clear_ms=0; s->marker_ms=0; s->departed=0; }
uint8_t Crossline_Update(CrosslineStop *s,uint8_t marker,uint8_t armed,uint32_t interval)
{
    if (!s->departed) {
        s->clear_ms=marker ? 0U : s->clear_ms+interval;
        if (s->clear_ms>=CROSSLINE_CLEAR_MS) s->departed=1;
    }
    s->marker_ms=(armed && s->departed && marker) ? s->marker_ms+interval : 0U;
    return s->marker_ms>=CROSSLINE_CONFIRM_MS;
}
