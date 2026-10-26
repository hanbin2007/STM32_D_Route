#ifndef IMU_H
#define IMU_H
#include <stdint.h>
typedef enum { IMU_OFFLINE,IMU_CALIBRATING,IMU_READY,IMU_ERROR } ImuStatus;
typedef struct {
    ImuStatus status;
    uint8_t address;
    uint32_t last_sample_ms;
    unsigned calibration_samples;
    float bias_z_dps;
    float yaw_deg;       /* Unwrapped, right-positive. Reset only at run start. */
    float yaw_rate_dps;
} ImuData;
void Imu_Init(void);
void Imu_Tick(uint32_t now,uint8_t wheels_stationary);
const ImuData *Imu_GetData(void);
uint8_t Imu_DataReady(const ImuData *data,uint32_t now);
void Imu_ResetYaw(void);
#endif
