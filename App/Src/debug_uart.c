#include "debug_uart.h"
#include "usart.h"
#include "board_config.h"
#include "imu.h"
#include <stdio.h>
static char buffer[512];
static uint32_t last_publish;
void DebugUart_Init(void) { last_publish=HAL_GetTick()-DEBUG_PERIOD_MS; }
void DebugUart_Publish(uint32_t now_ms, const char *state, unsigned fault,
                       const Gray8Data *gray, const EncoderData *encoder,
                       const int16_t *pwm, const RouteState *route,
                       const OdomData *odom, uint8_t finish_armed)
{
    int length;
    const ImuData *imu=Imu_GetData();
    unsigned raw_mask=0;
    if ((uint32_t)(now_ms-last_publish) < DEBUG_PERIOD_MS || huart1.gState != HAL_UART_STATE_READY) return;
    last_publish=now_ms;
    for (unsigned i=0; i<8; ++i) if (gray->raw[i]) raw_mask |= 1U<<i;
    length=snprintf(buffer,sizeof(buffer),
        "t=%lu state=%s fault=%u raw=%02X line=%02X n=%u pos10=%d marker=%u s=%lu "
        "cnt=%ld,%ld,%ld,%ld v=%d,%d,%d,%d pwm=%d,%d,%d,%d "
        "stage=%s side=%d,%d seg=%d,%d goal=%d,%d cmd=%d,%d arm=%u imu=%u cal=%u yaw10=%d rate10=%d ref10=%d off10=%d\r\n",
        (unsigned long)now_ms,state,fault,raw_mask,(unsigned)gray->active_mask,
        (unsigned)gray->active_count,(int)(gray->position*10.0f),(unsigned)gray->marker,
        (unsigned long)encoder->travel_mm,
        (long)encoder->count[0],(long)encoder->count[1],(long)encoder->count[2],(long)encoder->count[3],
        (int)encoder->speed_mm_s[0],(int)encoder->speed_mm_s[1],
        (int)encoder->speed_mm_s[2],(int)encoder->speed_mm_s[3],
        (int)pwm[0],(int)pwm[1],(int)pwm[2],(int)pwm[3],Route_SegmentName(route),
        (int)odom->left_mm,(int)odom->right_mm,
        (int)route->progress_left_mm,(int)route->progress_right_mm,
        (int)route->goal_left_mm,(int)route->goal_right_mm,
        (int)route->command_left_mm_s,(int)route->command_right_mm_s,(unsigned)finish_armed,
        (unsigned)imu->status,imu->calibration_samples,(int)(imu->yaw_deg*10),(int)(imu->yaw_rate_dps*10),
        (int)(route->heading_target_deg*10),(int)(route->boundary_offset_deg*10));
    if (length > 0 && (unsigned)length < sizeof(buffer))
        (void)HAL_UART_Transmit_IT(&huart1,(uint8_t *)buffer,(uint16_t)length);
}
