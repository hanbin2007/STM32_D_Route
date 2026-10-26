#include "hmi.h"
#include "oled.h"

static uint32_t t_start;
static uint32_t t_stop;
static uint8_t  running;
static uint8_t  has_time;
static uint32_t last_draw;
static uint8_t extra;

static void pad20(char *s)
{
    uint8_t i = 0;
    while (s[i]) i++;
    while (i < 20) s[i++] = ' ';
    s[20] = 0;
}

/* 12345 -> "12345" */
static void fmt_int(int32_t v, char *s)
{
    char tmp[12];
    uint8_t n = 0;
    uint8_t neg = 0;

    uint32_t magnitude=(v<0) ? 0U-(uint32_t)v : (uint32_t)v;
    if (v < 0) neg = 1;
    if (magnitude == 0U) tmp[n++]='0';
    while (magnitude > 0U) { tmp[n++]=(char)('0'+magnitude%10U); magnitude/=10U; }

    uint8_t i = 0;
    if (neg) s[i++] = '-';
    while (n > 0) s[i++] = tmp[--n];
    s[i] = 0;
}

/* milliseconds -> "00:12.34" */
static void fmt_time(uint32_t ms, char *s)
{
    uint32_t m   = ms / 60000u;
    uint32_t sec = (ms / 1000u) % 60u;
    uint32_t cs  = (ms / 10u) % 100u;

    s[0] = (char)('0' + (m / 10u) % 10u);
    s[1] = (char)('0' + m % 10u);
    s[2] = ':';
    s[3] = (char)('0' + sec / 10u);
    s[4] = (char)('0' + sec % 10u);
    s[5] = '.';
    s[6] = (char)('0' + cs / 10u);
    s[7] = (char)('0' + cs % 10u);
    s[8] = 0;
}

static void append(char *dst, uint8_t *pos, const char *src)
{
    while (*src && *pos < 20) dst[(*pos)++] = *src++;
    dst[*pos] = 0;
}

void HMI_Init(void)
{
    t_start = 0;
    t_stop = 0;
    running = 0;
    has_time = 0;
    last_draw = 0;
    extra = 0;

    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "S:IDLE");
    OLED_ShowString(0, 2, "T:00:00.00");
    OLED_ShowString(0, 4, "D:0mm");
    OLED_ShowString(0, 6, "V:0mm/s");
}

void HMI_Start(uint32_t now_ms)
{
    t_start = now_ms;
    t_stop = now_ms;
    running = 1;
    has_time = 0;
}

void HMI_Stop(uint32_t now_ms)
{
    if (running) {
        t_stop = now_ms;
        running = 0;
        has_time = 1;
    }
}

void HMI_Tick(uint32_t now_ms, const char *state, uint8_t fault,
              float dist_mm, float speed_mm_s)
{
    char buf[24];
    char num[16];
    uint8_t p;

    OLED_Service();
    if ((uint32_t)(now_ms - last_draw) < 100u) return;
    last_draw = now_ms;

    uint32_t elapsed;
    if (has_time)      elapsed = t_stop - t_start;
    else if (running)  elapsed = now_ms - t_start;
    else               elapsed = 0;

    /* time line, every 100 ms */
    fmt_time(elapsed, num);
    p = 0; buf[0] = 0;
    append(buf, &p, "T:");
    append(buf, &p, num);
    pad20(buf);
    OLED_ShowString(0, 2, buf);

    /* one more line per tick: state / distance / speed (full screen in 300 ms) */
    switch (extra) {
    case 0:
        p = 0; buf[0] = 0;
        append(buf, &p, "S:");
        append(buf, &p, state);
        pad20(buf);
        OLED_ShowString(0, 0, buf);
        break;
    case 1:
        fmt_int((int32_t)dist_mm, num);
        p = 0; buf[0] = 0;
        append(buf, &p, "D:");
        append(buf, &p, num);
        append(buf, &p, "mm");
        pad20(buf);
        OLED_ShowString(0, 4, buf);
        break;
    default:
        p = 0; buf[0] = 0;
        if (fault != 0u) {
            fmt_int((int32_t)fault, num);
            append(buf, &p, "F:");
            append(buf, &p, num);
        } else {
            fmt_int((int32_t)speed_mm_s, num);
            append(buf, &p, "V:");
            append(buf, &p, num);
            append(buf, &p, "mm/s");
        }
        pad20(buf);
        OLED_ShowString(0, 6, buf);
        break;
    }
    if (++extra > 2u) extra = 0u;
}
