/* Dedicated software I2C: board U7 SCL=PB10, SDA=PB11. The OLED uses
 * PB6/PB7 separately. Open-drain with board 3.3 V pull-ups; no IRQ masking.
 * Each transfer has a total 4 ms deadline, including clock stretching. */
#include "imu_bus.h"
#include "main.h"
#define IMU_SCL GPIO_PIN_10
#define IMU_SDA GPIO_PIN_11
static uint32_t transfer_start;
static uint8_t expired;
static void delay_half(void)
{
    const uint32_t start=DWT->CYCCNT;
    const uint32_t cycles=(SystemCoreClock/1000000U)*5U;
    while ((uint32_t)(DWT->CYCCNT-start)<cycles) { __NOP(); }
}
static uint8_t in_time(void)
{
    if ((uint32_t)(DWT->CYCCNT-transfer_start)>=(SystemCoreClock/1000U)*4U) expired=1;
    return !expired;
}
static void sda(uint8_t high) { HAL_GPIO_WritePin(GPIOB,IMU_SDA,high?GPIO_PIN_SET:GPIO_PIN_RESET); }
static void scl_low(void) { HAL_GPIO_WritePin(GPIOB,IMU_SCL,GPIO_PIN_RESET); }
static uint8_t scl_high(void)
{
    HAL_GPIO_WritePin(GPIOB,IMU_SCL,GPIO_PIN_SET);
    while (HAL_GPIO_ReadPin(GPIOB,IMU_SCL)==GPIO_PIN_RESET)
        if (!in_time()) return 0;
    delay_half();
    return in_time();
}
static uint8_t start(void)
{
    sda(1); delay_half();
    if (!scl_high()) return 0;
    if (HAL_GPIO_ReadPin(GPIOB,IMU_SDA)==GPIO_PIN_RESET) return 0;
    sda(0); delay_half(); scl_low(); delay_half(); return in_time();
}
static void stop(void)
{
    scl_low(); sda(0); delay_half();
    if (!expired) (void)scl_high();
    sda(1); delay_half();
    HAL_GPIO_WritePin(GPIOB,IMU_SCL,GPIO_PIN_SET);
}
static uint8_t put(uint8_t value)
{
    for (unsigned bit=0;bit<8U;++bit) {
        sda((value&0x80U)!=0U); delay_half();
        if (!scl_high()) return 0;
        scl_low(); delay_half(); value=(uint8_t)(value<<1);
    }
    sda(1); delay_half();
    if (!scl_high()) return 0;
    const uint8_t ack=HAL_GPIO_ReadPin(GPIOB,IMU_SDA)==GPIO_PIN_RESET;
    scl_low(); delay_half(); return ack && in_time();
}
static uint8_t get(uint8_t *value,uint8_t ack)
{
    uint8_t byte=0;
    sda(1);
    for (unsigned bit=0;bit<8U;++bit) {
        delay_half(); if (!scl_high()) return 0;
        byte=(uint8_t)((byte<<1)|(HAL_GPIO_ReadPin(GPIOB,IMU_SDA)==GPIO_PIN_SET?1U:0U));
        scl_low(); delay_half();
    }
    sda(!ack); delay_half(); if (!scl_high()) return 0;
    scl_low(); delay_half(); sda(1); *value=byte; return in_time();
}
void ImuBus_Init(void)
{
    GPIO_InitTypeDef gpio={0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    CoreDebug->DEMCR|=CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL|=DWT_CTRL_CYCCNTENA_Msk;
    HAL_GPIO_WritePin(GPIOB,IMU_SCL|IMU_SDA,GPIO_PIN_SET);
    gpio.Pin=IMU_SCL|IMU_SDA; gpio.Mode=GPIO_MODE_OUTPUT_OD;
    gpio.Speed=GPIO_SPEED_FREQ_HIGH; gpio.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB,&gpio);
}
void ImuBus_BootDelay(uint32_t milliseconds) { HAL_Delay(milliseconds); }
uint8_t ImuBus_Write(uint8_t address,uint8_t reg,uint8_t value)
{
    transfer_start=DWT->CYCCNT; expired=0;
    const uint8_t ok=start() && put((uint8_t)(address<<1)) && put(reg) && put(value);
    stop(); return ok && !expired;
}
uint8_t ImuBus_Read(uint8_t address,uint8_t reg,uint8_t *data,unsigned count)
{
    if (!data || count==0U || count>15U) return 0;
    transfer_start=DWT->CYCCNT; expired=0;
    uint8_t ok=start() && put((uint8_t)(address<<1)) && put(reg) && start() && put((uint8_t)((address<<1)|1U));
    for (unsigned i=0;ok && i<count;++i) ok=get(&data[i],i+1U<count);
    stop(); return ok && !expired;
}
