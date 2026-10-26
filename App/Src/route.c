#include "route.h"
#include "board_config.h"
#include <math.h>
#include <string.h>
#define RAD_PER_DEG 0.01745329252f
static const float straight_mm[4]={ROUTE_FIRST_MM,ROUTE_SIDE_MM,ROUTE_TOP_MM,ROUTE_SIDE_MM};
static const char *const names[9]={"S1","A","S2","B","S3","C","S4","D","FINISH"};
static float clamp(float x,float lo,float hi) { return x<lo?lo:(x>hi?hi:x); }
uint8_t Route_InArc(const RouteState *r)
{
    return r->result==ROUTE_RUNNING && r->segment<8U && (r->segment&1U);
}
static float center_goal(const RouteState *r) { return (r->goal_left_mm+r->goal_right_mm)*0.5f; }
static float center_progress(const RouteState *r) { return (r->progress_left_mm+r->progress_right_mm)*0.5f; }
static void enter_segment(RouteState *r,const OdomData *odom,uint32_t now)
{
    r->start_left_mm=odom->left_mm; r->start_right_mm=odom->right_mm;
    r->progress_left_mm=0; r->progress_right_mm=0; r->segment_start_ms=now;
    r->boundary_offset_deg=0;
    r->heading_target_deg=r->heading_origin_deg+(float)((r->segment+1U)/2U)*90.0f;
    if (!Route_InArc(r)) {
        const float distance=r->segment==8U?ROUTE_FINISH_MAX_MM:straight_mm[r->segment/2U];
        r->goal_left_mm=distance*(1.0f+ROUTE_STRAIGHT_TRIM);
        r->goal_right_mm=distance*(1.0f-ROUTE_STRAIGHT_TRIM);
    } else {
        /* Reference distances for diagnostics/feed-forward, NOT yaw completion.
         * IMU angle is authoritative; wheel slip changes actual arc travel. */
        const float angle=90.0f*RAD_PER_DEG;
        const float half_track=ROUTE_EFFECTIVE_TRACK_MM*0.5f*ROUTE_TURN_DIFFERENTIAL_GAIN;
        r->goal_left_mm=(ROUTE_ARC_RADIUS_MM+half_track)*angle;
        r->goal_right_mm=(ROUTE_ARC_RADIUS_MM-half_track)*angle;
    }
}
void Route_Start(RouteState *r,const OdomData *odom,uint32_t now,const ImuData *imu)
{
    memset(r,0,sizeof(*r)); r->result=ROUTE_RUNNING; r->last_update_ms=now;
    if (!(WHEEL_DIAMETER_MM>0 && WHEEL_COUNTS_PER_REV>0 && ROUTE_EFFECTIVE_TRACK_MM>0 &&
          ROUTE_ARC_RADIUS_MM>0 && ROUTE_TURN_DIFFERENTIAL_GAIN>0 &&
          ROUTE_STRAIGHT_SPEED_MM_S>0 && ROUTE_ARC_CENTER_SPEED_MM_S>0 &&
          ROUTE_ARC_CENTER_SPEED_MM_S<SPEED_MAX_MM_S && ROUTE_FINISH_SPEED_MM_S>0 &&
          ROUTE_END_SPEED_MM_S>0 && ROUTE_FIRST_MM>0 && ROUTE_SIDE_MM>0 && ROUTE_TOP_MM>0 &&
          fabsf(ROUTE_STRAIGHT_TRIM)<1 && ROUTE_LINE_KP>=0 && ROUTE_LINE_MAX_MM_S>0 &&
          fabsf(ROUTE_LINE_DIRECTION)==1 && HEADING_STRAIGHT_KP>0 && HEADING_STRAIGHT_KD>=0 &&
          HEADING_STRAIGHT_MAX_MM_S>0 && HEADING_STRAIGHT_MAX_MM_S<ROUTE_END_SPEED_MM_S &&
          HEADING_STRAIGHT_MAX_MM_S<ROUTE_FINISH_SPEED_MM_S &&
          ROUTE_STRAIGHT_SPEED_MM_S+HEADING_STRAIGHT_MAX_MM_S<=SPEED_MAX_MM_S &&
          HEADING_BOUNDARY_MAX_DEG>=0 && HEADING_BOUNDARY_SLEW_DPS>0 &&
          HEADING_TURN_ANGLE_KP>0 && HEADING_TURN_RATE_KP>=0 && HEADING_TURN_REVERSE_MAX_DPS>0 &&
          HEADING_TURN_TOLERANCE_DEG>0 && HEADING_TURN_EXIT_RATE_DPS>0 &&
          ROUTE_FINISH_ARM_MM>=0 && ROUTE_FINISH_MAX_MM>ROUTE_FINISH_ARM_MM)) {
        r->result=ROUTE_CONFIG_ERROR; return;
    }
    if (!Imu_DataReady(imu,now)) { r->result=ROUTE_IMU_ERROR; return; }
    r->heading_origin_deg=imu->yaw_deg;
    r->heading_deg=imu->yaw_deg; r->yaw_rate_dps=imu->yaw_rate_dps;
    enter_segment(r,odom,now);
}
uint8_t Route_FinishArmed(const RouteState *r,const OdomData *odom)
{
    return r->result==ROUTE_RUNNING && r->segment==8U &&
        odom->left_mm-r->start_left_mm>=ROUTE_FINISH_ARM_MM &&
        odom->right_mm-r->start_right_mm>=ROUTE_FINISH_ARM_MM;
}
static uint8_t segment_complete(const RouteState *r)
{
    if (Route_InArc(r))
        return fabsf(r->heading_target_deg-r->heading_deg)<=HEADING_TURN_TOLERANCE_DEG &&
               fabsf(r->yaw_rate_dps)<=HEADING_TURN_EXIT_RATE_DPS;
    return center_goal(r)-center_progress(r)<=ROUTE_GOAL_TOLERANCE_MM;
}
RouteResult Route_Update(RouteState *r,const OdomData *odom,uint32_t now,
                         uint8_t finish_detected,const Gray8Data *line,const ImuData *imu)
{
    const uint32_t elapsed=(uint32_t)(now-r->last_update_ms);
    const float dt=(float)(elapsed>CONTROL_MAX_INTERVAL_MS?CONTROL_MAX_INTERVAL_MS:elapsed)/1000.0f;
    float center_speed,turn;
    r->last_update_ms=now;
    r->command_left_mm_s=0; r->command_right_mm_s=0;
    if (r->result!=ROUTE_RUNNING) return r->result;
    if (!Imu_DataReady(imu,now)) return r->result=ROUTE_IMU_ERROR;
    r->heading_deg=imu->yaw_deg; r->yaw_rate_dps=imu->yaw_rate_dps;
    r->progress_left_mm=odom->left_mm-r->start_left_mm;
    r->progress_right_mm=odom->right_mm-r->start_right_mm;
    if (finish_detected && Route_FinishArmed(r,odom)) return r->result=ROUTE_COMPLETE;
    if (segment_complete(r)) {
        if (r->segment==8U) return r->result=ROUTE_FINISH_MISSING;
        ++r->segment; enter_segment(r,odom,now);
    }
    const uint8_t arc=Route_InArc(r);
    const float nominal_speed=arc?ROUTE_ARC_CENTER_SPEED_MM_S:
        (r->segment==8U?ROUTE_FINISH_SPEED_MM_S:ROUTE_STRAIGHT_SPEED_MM_S);
    if ((uint32_t)(now-r->segment_start_ms)>(uint32_t)(center_goal(r)/nominal_speed*1000.0f)+ROUTE_SEGMENT_MARGIN_MS)
        return r->result=ROUTE_TIMEOUT;
    if (!arc) {
        float wanted_offset=0;
        center_speed=nominal_speed;
        if (r->segment!=8U && center_goal(r)-center_progress(r)<ROUTE_SLOW_ZONE_MM)
            center_speed=ROUTE_END_SPEED_MM_S;
        if (line && line->line_valid && !line->marker && isfinite(line->position))
            wanted_offset=clamp(ROUTE_LINE_DIRECTION*ROUTE_LINE_KP*line->position/ROUTE_LINE_MAX_MM_S,-1,1)*HEADING_BOUNDARY_MAX_DEG;
        r->boundary_offset_deg+=clamp(wanted_offset-r->boundary_offset_deg,
            -HEADING_BOUNDARY_SLEW_DPS*dt,HEADING_BOUNDARY_SLEW_DPS*dt);
        const float target=r->heading_target_deg+r->boundary_offset_deg;
        turn=center_speed*ROUTE_STRAIGHT_TRIM+
            HEADING_STRAIGHT_KP*(target-r->heading_deg)-HEADING_STRAIGHT_KD*r->yaw_rate_dps;
        turn=clamp(turn,-HEADING_STRAIGHT_MAX_MM_S,HEADING_STRAIGHT_MAX_MM_S);
        r->yaw_rate_target_dps=0; /* Straight PD uses the raw yaw rate for damping. */
    } else {
        /* Right-positive yaw. Decelerate angular and forward speed together,
         * preserving the nominal radius; allow a small left correction if
         * inertia overshoots the planned 90/180/270/360-degree heading. */
        const float max_rate=ROUTE_ARC_CENTER_SPEED_MM_S/ROUTE_ARC_RADIUS_MM/RAD_PER_DEG;
        r->yaw_rate_target_dps=clamp(HEADING_TURN_ANGLE_KP*(r->heading_target_deg-r->heading_deg),
                                   -HEADING_TURN_REVERSE_MAX_DPS,max_rate);
        center_speed=ROUTE_ARC_RADIUS_MM*fabsf(r->yaw_rate_target_dps)*RAD_PER_DEG;
        turn=r->yaw_rate_target_dps*RAD_PER_DEG*ROUTE_EFFECTIVE_TRACK_MM*0.5f*ROUTE_TURN_DIFFERENTIAL_GAIN+
            HEADING_TURN_RATE_KP*(r->yaw_rate_target_dps-r->yaw_rate_dps);
        turn=clamp(turn,-(SPEED_MAX_MM_S-center_speed),SPEED_MAX_MM_S-center_speed);
    }
    r->command_left_mm_s=center_speed+turn;
    r->command_right_mm_s=center_speed-turn;
    return r->result;
}
const char *Route_SegmentName(const RouteState *r) { return r->segment<9U?names[r->segment]:"INVALID"; }
