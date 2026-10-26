#include "motor_test.h"
#include "main.h"
#include "usart.h"
#include "motor.h"
#include "encoder.h"
#include "speed_control.h"
#include "board_config.h"
#include <stdio.h>
#include <string.h>

static void uart_print(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), 1000U);
}

static void wait_ms(uint32_t duration_ms)
{
    uint32_t start = HAL_GetTick();
    uint32_t last = start;
    while ((uint32_t)(HAL_GetTick() - start) < duration_ms) {
        uint32_t now = HAL_GetTick();
        uint32_t interval = (uint32_t)(now - last);
        if (interval > 0U) {
            Encoder_Update(interval);
            last = now;
        }
    }
}

void MotorTest_Run(void)
{
    char line[160];
    const float mm_per_count =
        3.14159265f * WHEEL_DIAMETER_MM / WHEEL_COUNTS_PER_REV;

    SpeedControl_Stop(0);
    Motor_StopAll(0);
    uart_print("LIFT CAR! MOTOR TEST STARTS IN 3S\r\n");
    wait_ms(3000U);
    uart_print("OPEN_BEGIN,id,pwm,speed_mm_s\r\n");

    for (unsigned id = 0U; id < MOTOR_COUNT; ++id) {
        for (int pwm = 0; pwm <= 800; pwm += 50) {
            int32_t before;
            int32_t after;
            float speed;
            Motor_SetPwm(id, (int16_t)pwm);
            wait_ms(200U);
            before = Encoder_GetData()->count[id];
            wait_ms(300U);
            after = Encoder_GetData()->count[id];
            speed = (float)(after - before) * mm_per_count * 1000.0f / 300.0f;
            (void)snprintf(line, sizeof(line), "OPEN,%u,%d,%d\r\n",
                           id, pwm, (int)speed);
            uart_print(line);
        }
        Motor_SetPwm(id, 0);
        wait_ms(300U);
    }

    SpeedControl_Stop(0);
    wait_ms(300U);
    uart_print("CLOSED_BEGIN,ms,v0,v1,v2,v3,pwm0,pwm1,pwm2,pwm3\r\n");
    {
        const float targets[MOTOR_COUNT] = {300.0f, 300.0f, 300.0f, 300.0f};
        float v_sum[MOTOR_COUNT] = {0};
        float pwm_sum[MOTOR_COUNT] = {0};
        unsigned samples = 0U;
        uint32_t start = HAL_GetTick();
        uint32_t last = start;
        uint32_t next_print = start;
        SpeedControl_SetTargets(targets);
        while ((uint32_t)(HAL_GetTick() - start) < 3000U) {
            uint32_t now = HAL_GetTick();
            uint32_t interval = (uint32_t)(now - last);
            const EncoderData *encoder;
            const int16_t *pwm;
            if (interval >= 10U) {
                Encoder_Update(interval);
                SpeedControl_Tick(Encoder_GetData()->speed_mm_s, interval);
                last = now;
            }
            if ((uint32_t)(now - next_print) >= 100U) {
                encoder = Encoder_GetData();
                pwm = SpeedControl_GetPwm();
                next_print = now;
                (void)snprintf(line, sizeof(line),
                    "CLOSED,%lu,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                    (unsigned long)(now - start),
                    (int)encoder->speed_mm_s[0], (int)encoder->speed_mm_s[1],
                    (int)encoder->speed_mm_s[2], (int)encoder->speed_mm_s[3],
                    (int)pwm[0], (int)pwm[1], (int)pwm[2], (int)pwm[3]);
                uart_print(line);
                if ((uint32_t)(now - start) >= 1000U) {
                    for (unsigned i = 0U; i < MOTOR_COUNT; ++i) {
                        v_sum[i] += encoder->speed_mm_s[i];
                        pwm_sum[i] += (float)pwm[i];
                    }
                    ++samples;
                }
            }
        }
        if (samples > 0U) {
            (void)snprintf(line, sizeof(line),
                "CLOSED_AVG,target=300,v=%d,%d,%d,%d,pwm=%d,%d,%d,%d\r\n",
                (int)(v_sum[0] / (float)samples),
                (int)(v_sum[1] / (float)samples),
                (int)(v_sum[2] / (float)samples),
                (int)(v_sum[3] / (float)samples),
                (int)(pwm_sum[0] / (float)samples),
                (int)(pwm_sum[1] / (float)samples),
                (int)(pwm_sum[2] / (float)samples),
                (int)(pwm_sum[3] / (float)samples));
            uart_print(line);
        }
    }

    SpeedControl_Stop(0);
    Motor_StopAll(0);
    uart_print("MOTOR_TEST_END\r\n");
}
