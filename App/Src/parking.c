#include "parking.h"
#include "board_config.h"
#include <math.h>
#include <string.h>

#define DEG2RAD 0.01745329252f
#define MIN_WHEEL_SPEED_MM_S 20.0f

static float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

static float absf(float x)
{
    return x < 0.0f ? -x : x;
}

static uint8_t config_valid(const ParkingConfig *c)
{
    return c &&
        c->approach_mm >= 0.0f &&
        c->approach_speed_mm_s > 0.0f &&
        c->approach_speed_mm_s <= SPEED_MAX_MM_S &&
        c->approach_end_speed_mm_s > 0.0f &&
        c->approach_end_speed_mm_s <= c->approach_speed_mm_s &&
        c->rotate_max_dps > 0.0f &&
        c->rotate_tolerance_deg > 0.0f &&
        c->rotate_exit_rate_dps > 0.0f &&
        c->rotate_kp_dps_per_deg > 0.0f &&
        c->rotate_kd_dps_per_dps >= 0.0f &&
        c->reverse_mm >= 0.0f &&
        c->reverse_speed_mm_s > 0.0f &&
        c->reverse_speed_mm_s <= SPEED_MAX_MM_S &&
        c->heading_kp_mm_s_per_deg > 0.0f &&
        c->heading_kd_mm_s_per_dps >= 0.0f &&
        c->heading_max_mm_s > 0.0f &&
        c->distance_tolerance_mm >= 0.0f &&
        c->stop_speed_mm_s >= 0.0f &&
        c->stop_settle_ms > 0U &&
        c->timeout_ms > 0U &&
        c->track_mm > 0.0f;
}

static void stop_command(Parking *p)
{
    p->command_left_mm_s = 0.0f;
    p->command_right_mm_s = 0.0f;
}

static void enter_state(Parking *p, ParkingState state, uint32_t now)
{
    stop_command(p);
    p->state = state;
    p->state_start_ms = now;
    p->settle_ms = 0;
}

static void snapshot_start(Parking *p, const OdomData *odom)
{
    p->start_left_mm = odom->left_mm;
    p->start_right_mm = odom->right_mm;
    p->progress_mm = 0.0f;
}

static float heading_turn(Parking *p, const ParkingConfig *c, const ImuData *imu)
{
    float turn;
    p->heading_error_deg = p->target_heading_deg - imu->yaw_deg;
    turn = c->heading_kp_mm_s_per_deg * p->heading_error_deg -
           c->heading_kd_mm_s_per_dps * imu->yaw_rate_dps;
    return clampf(turn, -c->heading_max_mm_s, c->heading_max_mm_s);
}

static float limit_differential(float turn, float speed)
{
    const float limit = absf(speed) - MIN_WHEEL_SPEED_MM_S;
    if (limit <= 0.0f) return 0.0f;
    return clampf(turn, -limit, limit);
}

void Parking_DefaultConfig(ParkingConfig *c)
{
    memset(c, 0, sizeof(*c));
    c->approach_mm = 650.0f;
    c->approach_speed_mm_s = 220.0f;
    c->approach_heading_deg = 180.0f;
    c->approach_slow_zone_mm = 80.0f;
    c->approach_end_speed_mm_s = 120.0f;
    c->rotate_heading_deg = 90.0f;
    c->rotate_max_dps = 60.0f;
    c->rotate_tolerance_deg = 2.0f;
    c->rotate_exit_rate_dps = 5.0f;
    c->rotate_kp_dps_per_deg = 2.5f;
    c->rotate_kd_dps_per_dps = 0.3f;
    c->reverse_mm = 273.5f;
    c->reverse_speed_mm_s = 100.0f;
    c->reverse_heading_deg = 90.0f;
    c->heading_kp_mm_s_per_deg = 4.0f;
    c->heading_kd_mm_s_per_dps = 0.5f;
    c->heading_max_mm_s = 60.0f;
    c->distance_tolerance_mm = 2.0f;
    c->stop_speed_mm_s = 15.0f;
    c->stop_settle_ms = 100U;
    c->timeout_ms = 15000U;
    c->track_mm = ROUTE_EFFECTIVE_TRACK_MM;
}

void Parking_Start(Parking *p, const ParkingConfig *c,
                   const OdomData *odom, const ImuData *imu, uint32_t now)
{
    memset(p, 0, sizeof(*p));
    p->state_start_ms = now;
    p->last_update_ms = now;
    p->last_left_mm = odom->left_mm;
    p->last_right_mm = odom->right_mm;
    p->target_heading_deg = c ? c->approach_heading_deg : 0.0f;
    p->target_distance_mm = c ? c->approach_mm : 0.0f;
    snapshot_start(p, odom);
    if (!config_valid(c)) {
        p->state = PARK_FAULT;
        p->result = PARK_CONFIG_ERROR;
        return;
    }
    if (!isfinite(imu->yaw_deg) || !isfinite(imu->yaw_rate_dps)) {
        p->state = PARK_FAULT;
        p->result = PARK_IMU_ERROR;
        return;
    }
    p->state = PARK_APPROACH;
    p->result = PARK_OK;
}

ParkingResult Parking_Update(Parking *p, const ParkingConfig *c,
                             const OdomData *odom, const ImuData *imu, uint32_t now)
{
    uint32_t elapsed;
    float dt;
    float dl;
    float dr;

    if (p->state == PARK_IDLE || p->state == PARK_DONE || p->state == PARK_FAULT) {
        stop_command(p);
        return p->result;
    }
    if (!isfinite(imu->yaw_deg) || !isfinite(imu->yaw_rate_dps)) {
        stop_command(p);
        p->state = PARK_FAULT;
        p->result = PARK_IMU_ERROR;
        return p->result;
    }

    elapsed = now - p->last_update_ms;
    if (elapsed == 0U) elapsed = 1U;
    dt = (float)elapsed / 1000.0f;
    if (dt > (float)CONTROL_MAX_INTERVAL_MS / 1000.0f)
        dt = (float)CONTROL_MAX_INTERVAL_MS / 1000.0f;

    dl = odom->left_mm - p->last_left_mm;
    dr = odom->right_mm - p->last_right_mm;
    p->speed_mm_s = (dl + dr) * 0.5f / dt;
    p->last_left_mm = odom->left_mm;
    p->last_right_mm = odom->right_mm;
    p->last_update_ms = now;
    p->progress_mm = ((odom->left_mm - p->start_left_mm) +
                      (odom->right_mm - p->start_right_mm)) * 0.5f;

    if ((uint32_t)(now - p->state_start_ms) >= c->timeout_ms) {
        stop_command(p);
        p->state = PARK_FAULT;
        p->result = PARK_TIMEOUT;
        return p->result;
    }

    switch (p->state) {
    case PARK_APPROACH: {
        float remaining = c->approach_mm - p->progress_mm;
        float speed;
        float turn;
        if (remaining <= c->distance_tolerance_mm) {
            enter_state(p, PARK_STOP, now);
            break;
        }
        speed = remaining < c->approach_slow_zone_mm ?
                c->approach_end_speed_mm_s : c->approach_speed_mm_s;
        turn = limit_differential(heading_turn(p, c, imu), speed);
        p->command_left_mm_s = speed + turn;
        p->command_right_mm_s = speed - turn;
        break;
    }
    case PARK_STOP:
        stop_command(p);
        if (absf(p->speed_mm_s) <= c->stop_speed_mm_s) {
            p->settle_ms += (uint32_t)(dt * 1000.0f);
        } else {
            p->settle_ms = 0U;
        }
        if (p->settle_ms >= c->stop_settle_ms) {
            p->target_heading_deg = c->rotate_heading_deg;
            enter_state(p, PARK_ROTATE, now);
        }
        break;
    case PARK_ROTATE: {
        float error = p->target_heading_deg - imu->yaw_deg;
        float omega;
        float turn_mm_s;
        p->heading_error_deg = error;
        if (absf(error) <= c->rotate_tolerance_deg &&
            absf(imu->yaw_rate_dps) <= c->rotate_exit_rate_dps) {
            snapshot_start(p, odom);
            p->target_heading_deg = c->reverse_heading_deg;
            p->target_distance_mm = c->reverse_mm;
            enter_state(p, PARK_REVERSE, now);
            break;
        }
        omega = c->rotate_kp_dps_per_deg * error -
                c->rotate_kd_dps_per_dps * imu->yaw_rate_dps;
        omega = clampf(omega, -c->rotate_max_dps, c->rotate_max_dps);
        turn_mm_s = omega * DEG2RAD * c->track_mm * 0.5f;
        p->command_left_mm_s = turn_mm_s;
        p->command_right_mm_s = -turn_mm_s;
        break;
    }
    case PARK_REVERSE: {
        float reverse_progress = -p->progress_mm;
        float speed;
        float turn;
        if (reverse_progress >= c->reverse_mm - c->distance_tolerance_mm) {
            enter_state(p, PARK_ALIGN, now);
            break;
        }
        speed = -c->reverse_speed_mm_s;
        turn = limit_differential(heading_turn(p, c, imu), speed);
        p->command_left_mm_s = speed + turn;
        p->command_right_mm_s = speed - turn;
        break;
    }
    case PARK_ALIGN:
        stop_command(p);
        if (absf(p->speed_mm_s) <= c->stop_speed_mm_s) {
            p->settle_ms += (uint32_t)(dt * 1000.0f);
        } else {
            p->settle_ms = 0U;
        }
        if (p->settle_ms >= c->stop_settle_ms)
            enter_state(p, PARK_DONE, now);
        break;
    case PARK_DONE:
        stop_command(p);
        break;
    default:
        stop_command(p);
        p->state = PARK_FAULT;
        p->result = PARK_CONFIG_ERROR;
        break;
    }
    return p->result;
}

const char *Parking_StateName(const Parking *p)
{
    switch (p->state) {
    case PARK_IDLE: return "P_IDLE";
    case PARK_APPROACH: return "P_APPROACH";
    case PARK_STOP: return "P_STOP";
    case PARK_ROTATE: return "P_ROTATE";
    case PARK_REVERSE: return "P_REVERSE";
    case PARK_ALIGN: return "P_ALIGN";
    case PARK_DONE: return "P_DONE";
    default: return "P_FAULT";
    }
}

const char *Parking_ResultName(const Parking *p)
{
    switch (p->result) {
    case PARK_OK: return "PARK_OK";
    case PARK_CONFIG_ERROR: return "PARK_CONFIG_ERROR";
    case PARK_IMU_ERROR: return "PARK_IMU_ERROR";
    case PARK_TIMEOUT: return "PARK_TIMEOUT";
    default: return "PARK_UNKNOWN";
    }
}
