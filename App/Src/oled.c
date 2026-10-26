/* Adapted from the teammate OLED_HMI package for the current board.
 * The display is optional and every poll sends a bounded chunk. */
#include "oled.h"
#include "main.h"
#include "font5x7.h"
#include <string.h>
#define OLED_ADDR 0x78U
#define OLED_CHUNK_BYTES 16U
#define OLED_SCL GPIO_PIN_6
#define OLED_SDA GPIO_PIN_7
static uint8_t pixels[8][128];
static uint8_t dirty_pages, tx_page, tx_col, present;

static void dly(void)
{
    for (volatile unsigned i=0;i<6U;++i) { __NOP(); }
}
static void scl(uint8_t value)
{
    if (value) GPIOB->BSRR=OLED_SCL; else GPIOB->BRR=OLED_SCL;
}
static void sda(uint8_t value)
{
    if (value) GPIOB->BSRR=OLED_SDA; else GPIOB->BRR=OLED_SDA;
}
static void i2c_start(void)
{
    sda(1); dly(); scl(1); dly(); sda(0); dly(); scl(0); dly();
}
static void i2c_stop(void)
{
    sda(0); dly(); scl(1); dly(); sda(1); dly();
}
static void i2c_write(uint8_t value)
{
    for (unsigned i=0;i<8U;++i) {
        sda((value&0x80U)!=0U); dly(); scl(1); dly(); scl(0); dly();
        value=(uint8_t)(value<<1);
    }
    sda(1); dly(); scl(1); dly(); scl(0); dly();
}
static void command(uint8_t value)
{
    i2c_start(); i2c_write(OLED_ADDR); i2c_write(0x00); i2c_write(value); i2c_stop();
}
static void set_pos(uint8_t page,uint8_t col)
{
    command((uint8_t)(0xB0U|page));
    command((uint8_t)(col&0x0FU));
    command((uint8_t)(0x10U|(col>>4)));
}
static uint8_t probe(void)
{
    GPIO_InitTypeDef gpio={0};
    uint8_t value=OLED_ADDR;
    i2c_start();
    for (unsigned i=0;i<8U;++i) {
        sda((value&0x80U)!=0U); dly(); scl(1); dly(); scl(0); dly();
        value=(uint8_t)(value<<1);
    }
    sda(1); dly(); scl(1); dly();
    gpio.Pin=OLED_SDA; gpio.Mode=GPIO_MODE_INPUT; gpio.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB,&gpio);
    const uint8_t ack=HAL_GPIO_ReadPin(GPIOB,OLED_SDA)==GPIO_PIN_RESET;
    scl(0); dly();
    gpio.Mode=GPIO_MODE_OUTPUT_OD; gpio.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&gpio);
    i2c_stop();
    return ack;
}
void OLED_Init(void)
{
    GPIO_InitTypeDef gpio={0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB,OLED_SCL|OLED_SDA,GPIO_PIN_SET);
    gpio.Pin=OLED_SCL|OLED_SDA; gpio.Mode=GPIO_MODE_OUTPUT_OD;
    gpio.Speed=GPIO_SPEED_FREQ_HIGH; gpio.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB,&gpio); /* board has external 4.7k pull-ups */
    scl(1); sda(1); dly();
    HAL_Delay(100U); /* boot only: allow display power-on reset to finish */
    present=probe();
    tx_page=0; tx_col=0;
    OLED_Clear();
    if (!present) return;
    static const uint8_t init[]={0xAE,0xD5,0x80,0xA8,0x3F,0xD3,0x00,0x40,
        0x8D,0x14,0x20,0x02,0xA1,0xC8,0xDA,0x12,0x81,0xCF,0xD9,0xF1,
        0xDB,0x40,0xA4,0xA6,0xAF};
    for (unsigned i=0;i<sizeof(init);++i) command(init[i]);
}
uint8_t OLED_GetBus(void) { return present ? 1U : 0U; }
void OLED_Clear(void)
{
    memset(pixels,0,sizeof(pixels));
    dirty_pages=0xFFU; tx_col=0;
}
void OLED_ShowString(uint8_t x,uint8_t page,const char *text)
{
    unsigned col=(unsigned)x*6U;
    if (page>=8U || col>=128U) return;
    while (*text && col+6U<=128U) {
        uint8_t c=(uint8_t)*text++;
        if (c<32U || c>127U) c='?';
        const uint8_t *glyph=&font5x7[(c-32U)*5U];
        for (unsigned i=0;i<5U;++i) pixels[page][col++]=glyph[i];
        pixels[page][col++]=0;
    }
    dirty_pages|=(uint8_t)(1U<<page);
    /* If a line changes mid-transfer, send it again from its first column. */
    if (tx_page==page) tx_col=0;
}
void OLED_Service(void)
{
    if (!present || !dirty_pages) return;
    if ((dirty_pages&(uint8_t)(1U<<tx_page))==0U) {
        for (tx_page=0;tx_page<8U;++tx_page)
            if (dirty_pages&(uint8_t)(1U<<tx_page)) break;
        tx_col=0;
    }
    set_pos(tx_page,tx_col);
    i2c_start(); i2c_write(OLED_ADDR); i2c_write(0x40);
    for (unsigned i=0;i<OLED_CHUNK_BYTES;++i) i2c_write(pixels[tx_page][tx_col+i]);
    i2c_stop();
    tx_col=(uint8_t)(tx_col+OLED_CHUNK_BYTES);
    if (tx_col==128U) {
        dirty_pages&=(uint8_t)~(1U<<tx_page);
        tx_col=0;
    }
}
