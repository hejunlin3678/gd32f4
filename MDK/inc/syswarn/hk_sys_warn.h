
#ifndef __HK_SYS_WARN_H__
#define __HK_SYS_WARN_H__

#if 0
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "gd32f4xx_gpio.h"
#include "i2c/hk_i2c.h"
#include "rtc/rtc.h"

#define I2C_SENSOR (I2C1)
#define BM8563_ADDR (0x51 << 1)
#define LM75BD_ADDR (0x48 << 1)	//0x90
#define TPA626_ADDR (0x44 << 1)

#define SLOT1_PRENST_PIN GPIO_PIN_0
#define SLOT1_PRENST_PORT GPIOE
#define SLOT1_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT2_PRENST_PIN GPIO_PIN_1
#define SLOT2_PRENST_PORT GPIOE
#define SLOT2_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT4_PRENST_PIN GPIO_PIN_2
#define SLOT4_PRENST_PORT GPIOE
#define SLOT4_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT5_PRENST_PIN GPIO_PIN_3
#define SLOT5_PRENST_PORT GPIOE
#define SLOT5_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT6_PRENST_PIN GPIO_PIN_4
#define SLOT6_PRENST_PORT GPIOE
#define SLOT6_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT7_PRENST_PIN GPIO_PIN_5
#define SLOT7_PRENST_PORT GPIOE
#define SLOT7_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT8_PRENST_PIN GPIO_PIN_6
#define SLOT8_PRENST_PORT GPIOE
#define SLOT8_PRENST_GPIO_CLK RCU_GPIOE

#define SLOT9_PRENST_PIN GPIO_PIN_7
#define SLOT9_PRENST_PORT GPIOE
#define SLOT9_PRENST_GPIO_CLK RCU_GPIOE

typedef enum
{
    SLOT1_PRENST = 0,
    SLOT2_PRENST = 1,
    SLOT4_PRENST,
    SLOT5_PRENST,
    SLOT6_PRENST,
    SLOT7_PRENST,
    SLOT8_PRENST,
    SLOT9_PRENST,
} SLOT_STATE;



#define SLOT_GPIO_NUM 8
static uint32_t SLOT_GPIO_PORT[SLOT_GPIO_NUM] = {SLOT1_PRENST_PORT, SLOT2_PRENST_PORT, SLOT4_PRENST_PORT, SLOT5_PRENST_PORT,
                                            SLOT6_PRENST_PORT, SLOT7_PRENST_PORT, SLOT8_PRENST_PORT, SLOT9_PRENST_PORT};
static uint32_t SLOT_GPIO_PIN[SLOT_GPIO_NUM] = {SLOT1_PRENST_PIN, SLOT2_PRENST_PIN, SLOT4_PRENST_PIN, SLOT5_PRENST_PIN,
                                           SLOT6_PRENST_PIN, SLOT7_PRENST_PIN, SLOT8_PRENST_PIN, SLOT9_PRENST_PIN};
static uint32_t SLOT_GPIO_CLK[SLOT_GPIO_NUM] = {SLOT1_PRENST_GPIO_CLK, SLOT2_PRENST_GPIO_CLK, SLOT4_PRENST_GPIO_CLK, SLOT5_PRENST_GPIO_CLK,
                                           SLOT6_PRENST_GPIO_CLK, SLOT7_PRENST_GPIO_CLK, SLOT8_PRENST_GPIO_CLK, SLOT9_PRENST_GPIO_CLK};


uint8_t getReqAddr(void);
uint8_t getSlot(uint8_t addr);
void get_slots_gpio_init(void);
uint8_t get_slots_status(uint8_t slot_id);
void sys_warn_task(void);

#endif

#endif

