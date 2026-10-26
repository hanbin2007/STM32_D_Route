/* MPU6050: +/-500 dps (65.5 LSB/dps), +/-2 g, DLPF=3,
 * sensor 200 Hz, host reads one fresh burst every 10 ms.
 * No DMP/absolute heading claim. Startup stationary calibration only. */
#include "imu.h"
#include "imu_bus.h"
#include "board_config.h"
#include "main.h"
#include <math.h>
#include <string.h>
static ImuData data;
static float sums[3],squares[3],previous_rate;
static uint8_t have_previous;
static int16_t signed_word(const uint8_t *p)
{
    uint16_t value=(uint16_t)(((uint16_t)p[0]<<8)|p[1]);
    return (int16_t)(value<32768U?(int32_t)value:(int32_t)value-65536);
}
static void reset_calibration(void)
{
    data.calibration_samples=0;
    memset(sums,0,sizeof(sums)); memset(squares,0,sizeof(squares));
}
void Imu_Init(void)
{
    uint8_t id=0;
    static const uint8_t settings[][2]={
        {0x6B,0x01},{0x6C,0x00},{0x19,0x04},{0x1A,0x03},
        {0x1B,0x08},{0x1C,0x00},{0x37,0x00},{0x38,0x01}
    };
    memset(&data,0,sizeof(data)); reset_calibration();
    previous_rate=0; have_previous=0;
    if (!(fabsf(IMU_YAW_SIGN)==1.0f && IMU_CALIBRATION_SAMPLES>0 &&
          IMU_MAX_SAMPLE_AGE_MS>=CONTROL_PERIOD_MS && IMU_CALIBRATION_VARIANCE_DPS2>0)) {
        data.status=IMU_ERROR; return;
    }
    ImuBus_Init();
    for (unsigned i=0;i<2U;++i) {
        uint8_t address=(uint8_t)(0x68U+i);
        if (ImuBus_Read(address,0x75,&id,1U) && id==0x68U) { data.address=address; break; }
    }
    if (!data.address) return;
    if (!ImuBus_Write(data.address,0x6B,0x80)) { data.status=IMU_ERROR; return; }
    ImuBus_BootDelay(100U); /* boot only, motors initialized/stopped */
    for (unsigned i=0;i<sizeof(settings)/sizeof(settings[0]);++i) {
        if (!ImuBus_Write(data.address,settings[i][0],settings[i][1])) { data.status=IMU_ERROR; return; }
    }
    ImuBus_BootDelay(50U);
    for (unsigned i=0;i<sizeof(settings)/sizeof(settings[0]);++i) {
        if (!ImuBus_Read(data.address,settings[i][0],&id,1U) || id!=settings[i][1]) {
            data.status=IMU_ERROR; return;
        }
    }
    data.last_sample_ms=HAL_GetTick();
    data.status=IMU_CALIBRATING;
}
uint8_t Imu_DataReady(const ImuData *value,uint32_t now)
{
    return value && value->status==IMU_READY &&
        (uint32_t)(now-value->last_sample_ms)<=IMU_MAX_SAMPLE_AGE_MS &&
        isfinite(value->yaw_deg) && isfinite(value->yaw_rate_dps);
}
void Imu_Tick(uint32_t now,uint8_t wheels_stationary)
{
    uint8_t bytes[15];
    if (data.status!=IMU_CALIBRATING && data.status!=IMU_READY) return;
    if (!ImuBus_Read(data.address,0x3A,bytes,sizeof(bytes))) { data.status=IMU_ERROR; return; }
    if (!(bytes[0]&1U)) {
        if ((uint32_t)(now-data.last_sample_ms)>IMU_MAX_SAMPLE_AGE_MS) data.status=IMU_ERROR;
        return;
    }
    const uint32_t elapsed=(uint32_t)(now-data.last_sample_ms);
    data.last_sample_ms=now;
    const float ax=(float)signed_word(&bytes[1])/16384.0f;
    const float ay=(float)signed_word(&bytes[3])/16384.0f;
    const float az=(float)signed_word(&bytes[5])/16384.0f;
    const float gyro[3]={(float)signed_word(&bytes[9])/65.5f,
                         (float)signed_word(&bytes[11])/65.5f,
                         (float)signed_word(&bytes[13])/65.5f};
    if (data.status==IMU_CALIBRATING) {
        /* Mount Z up and leave the chassis still. Variance catches movement
         * during the window; constant slow hand rotation is not observable
         * separately from bias, so the user must keep it stationary. */
        if (!wheels_stationary || fabsf(ax)>0.20f || fabsf(ay)>0.20f || az<0.9f || az>1.1f ||
            fabsf(gyro[0])>30 || fabsf(gyro[1])>30 || fabsf(gyro[2])>30 ||
            elapsed>IMU_MAX_SAMPLE_AGE_MS) { reset_calibration(); return; }
        ++data.calibration_samples;
        for (unsigned i=0;i<3U;++i) { sums[i]+=gyro[i]; squares[i]+=gyro[i]*gyro[i]; }
        if (data.calibration_samples>=IMU_CALIBRATION_SAMPLES) {
            const float n=(float)data.calibration_samples;
            for (unsigned i=0;i<3U;++i) {
                const float mean=sums[i]/n;
                if (squares[i]/n-mean*mean>IMU_CALIBRATION_VARIANCE_DPS2) { reset_calibration(); return; }
            }
            data.bias_z_dps=sums[2]/n;
            data.yaw_deg=0; data.yaw_rate_dps=0; data.status=IMU_READY;
            previous_rate=0; have_previous=0;
        }
        return;
    }
    if (!elapsed || elapsed>IMU_MAX_SAMPLE_AGE_MS) { data.status=IMU_ERROR; return; }
    const float rate=IMU_YAW_SIGN*(gyro[2]-data.bias_z_dps);
    data.yaw_deg+=(have_previous?(previous_rate+rate)*0.5f:rate)*(float)elapsed/1000.0f;
    previous_rate=rate; have_previous=1; data.yaw_rate_dps=rate;
}
const ImuData *Imu_GetData(void) { return &data; }
void Imu_ResetYaw(void)
{
    data.yaw_deg=0;
    previous_rate=data.yaw_rate_dps; have_previous=1;
}
