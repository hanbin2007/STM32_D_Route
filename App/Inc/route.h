#ifndef ROUTE_H
#define ROUTE_H
#include <stdint.h>
#include "odometry.h"
#include "gray8.h"
#include "imu.h"
typedef enum { ROUTE_RUNNING,ROUTE_COMPLETE,ROUTE_TIMEOUT,ROUTE_FINISH_MISSING,
               ROUTE_CONFIG_ERROR,ROUTE_IMU_ERROR } RouteResult;
typedef struct {
    uint8_t segment;
    uint32_t segment_start_ms,last_update_ms;
    float start_left_mm,start_right_mm;
    float goal_left_mm,goal_right_mm;
    float progress_left_mm,progress_right_mm;
    float command_left_mm_s,command_right_mm_s;
    float heading_origin_deg,heading_target_deg;
    float heading_deg,yaw_rate_dps,yaw_rate_target_dps;
    float boundary_offset_deg;
    RouteResult result;
} RouteState;
void Route_Start(RouteState *route,const OdomData *odom,uint32_t now,const ImuData *imu);
RouteResult Route_Update(RouteState *route,const OdomData *odom,uint32_t now,
                         uint8_t finish_detected,const Gray8Data *line,const ImuData *imu);
uint8_t Route_InArc(const RouteState *route);
uint8_t Route_FinishArmed(const RouteState *route,const OdomData *odom);
const char *Route_SegmentName(const RouteState *route);
#endif
