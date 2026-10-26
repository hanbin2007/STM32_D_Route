#include "parking.h"
#include <math.h>
#include <stdio.h>

#define DT_S 0.01f
#define RAD2DEG 57.2957795131f

int main(void)
{
    ParkingConfig cfg;
    Parking parking;
    OdomData odom = {0};
    ImuData imu = {0};
    unsigned seen_rotate = 0;
    unsigned seen_reverse = 0;
    unsigned i;

    Parking_DefaultConfig(&cfg);
    imu.yaw_deg = cfg.approach_heading_deg;
    imu.yaw_rate_dps = 0.0f;

    Parking_Start(&parking, &cfg, &odom, &imu, 0U);
    if (parking.state != PARK_APPROACH || parking.result != PARK_OK) {
        printf("start failed: %s %s\n", Parking_StateName(&parking),
               Parking_ResultName(&parking));
        return 1;
    }

    for (i = 0; i < 3000U; ++i) {
        uint32_t now = (uint32_t)(i + 1U) * 10U;
        float yaw_rate;
        Parking_Update(&parking, &cfg, &odom, &imu, now);
        if (parking.state == PARK_DONE || parking.state == PARK_FAULT) break;
        if (parking.state == PARK_ROTATE) seen_rotate = 1U;
        if (parking.state == PARK_REVERSE) seen_reverse = 1U;

        odom.left_mm += parking.command_left_mm_s * DT_S;
        odom.right_mm += parking.command_right_mm_s * DT_S;
        odom.center_mm = (odom.left_mm + odom.right_mm) * 0.5f;
        yaw_rate = (parking.command_left_mm_s - parking.command_right_mm_s) /
                   cfg.track_mm * RAD2DEG;
        imu.yaw_deg += yaw_rate * DT_S;
        imu.yaw_rate_dps = yaw_rate;
    }

    if (parking.state != PARK_DONE || parking.result != PARK_OK) {
        printf("parking failed: %s %s yaw=%.2f odom=(%.2f,%.2f)\n",
               Parking_StateName(&parking), Parking_ResultName(&parking),
               imu.yaw_deg, odom.left_mm, odom.right_mm);
        return 1;
    }
    if (!seen_rotate || !seen_reverse) {
        printf("missing states: rotate=%u reverse=%u\n", seen_rotate, seen_reverse);
        return 1;
    }
    if (fabsf(imu.yaw_deg - cfg.reverse_heading_deg) > 3.0f) {
        printf("heading error: %.2f\n", imu.yaw_deg);
        return 1;
    }
    if (fabsf(odom.center_mm - (cfg.approach_mm - cfg.reverse_mm)) > 5.0f) {
        printf("distance error: %.2f expected %.2f\n", odom.center_mm,
               cfg.approach_mm - cfg.reverse_mm);
        return 1;
    }

    printf("PASS yaw=%.2f odom=(%.2f,%.2f)\n", imu.yaw_deg,
           odom.left_mm, odom.right_mm);
    return 0;
}
