#ifndef PARKING_H
#define PARKING_H

#include <stdint.h>
#include "odometry.h"
#include "imu.h"

typedef enum {
    PARK_IDLE = 0,
    PARK_APPROACH,
    PARK_STOP,
    PARK_ROTATE,
    PARK_REVERSE,
    PARK_ALIGN,
    PARK_DONE,
    PARK_FAULT
} ParkingState;

typedef enum {
    PARK_OK = 0,
    PARK_CONFIG_ERROR,
    PARK_IMU_ERROR,
    PARK_TIMEOUT
} ParkingResult;

typedef struct {
    float approach_mm;
    float approach_speed_mm_s;
    float approach_heading_deg;
    float approach_slow_zone_mm;
    float approach_end_speed_mm_s;
    float rotate_heading_deg;
    float rotate_max_dps;
    float rotate_tolerance_deg;
    float rotate_exit_rate_dps;
    float rotate_kp_dps_per_deg;
    float rotate_kd_dps_per_dps;
    float reverse_mm;
    float reverse_speed_mm_s;
    float reverse_heading_deg;
    float heading_kp_mm_s_per_deg;
    float heading_kd_mm_s_per_dps;
    float heading_max_mm_s;
    float distance_tolerance_mm;
    float stop_speed_mm_s;
    uint32_t stop_settle_ms;
    uint32_t timeout_ms;
    float track_mm;
} ParkingConfig;

typedef struct {
    ParkingState state;
    ParkingResult result;
    uint32_t state_start_ms;
    uint32_t last_update_ms;
    float start_left_mm;
    float start_right_mm;
    float last_left_mm;
    float last_right_mm;
    float target_heading_deg;
    float target_distance_mm;
    float progress_mm;
    float speed_mm_s;
    float heading_error_deg;
    float command_left_mm_s;
    float command_right_mm_s;
    uint32_t settle_ms;
} Parking;

void Parking_DefaultConfig(ParkingConfig *config);
void Parking_Start(Parking *parking, const ParkingConfig *config,
                   const OdomData *odom, const ImuData *imu, uint32_t now);
ParkingResult Parking_Update(Parking *parking, const ParkingConfig *config,
                             const OdomData *odom, const ImuData *imu, uint32_t now);
const char *Parking_StateName(const Parking *parking);
const char *Parking_ResultName(const Parking *parking);

#endif
