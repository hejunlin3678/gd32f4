/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   hk_adc.h
 **
 ** \brief  hk adc define
 **
 ** \author
 **
 ******************************************************************************/
#ifndef __HK_ADC1_H__
#define __HK_ADC1_H__

#include <stdio.h>
#include "gd32f4xx.h"
#include "systick.h"
#include "gd32f4xx_adc.h"
#include "gd32f4xx_rcu.h"

// 根据功能填充采集数&引脚进行初始化
#define ADC_NUMS (6)

#define ADC_12V_MXM_RCU RCU_GPIOA
#define ADC_12V_MXM_PORT GPIOA
#define ADC_12V_MXM_PIN GPIO_PIN_0
#define ADC_12V_MXM_CHANNEL ADC_CHANNEL_0

#define ADC_12V_CPU_RCU RCU_GPIOC
#define ADC_12V_CPU_PORT GPIOC
#define ADC_12V_CPU_PIN GPIO_PIN_0
#define ADC_12V_CPU_CHANNEL ADC_CHANNEL_10

#define ADC_3V3_CPU_RCU RCU_GPIOC
#define ADC_3V3_CPU_PORT GPIOC
#define ADC_3V3_CPU_PIN GPIO_PIN_2
#define ADC_3V3_CPU_CHANNEL ADC_CHANNEL_12

#define ADC_5V_BOARD_RCU RCU_GPIOA
#define ADC_5V_BOARD_PORT GPIOA
#define ADC_5V_BOARD_PIN GPIO_PIN_3
#define ADC_5V_BOARD_CHANNEL ADC_CHANNEL_3

#define ADC_3V3_BOARD_RCU RCU_GPIOC
#define ADC_3V3_BOARD_PORT GPIOC
#define ADC_3V3_BOARD_PIN GPIO_PIN_3
#define ADC_3V3_BOARD_CHANNEL ADC_CHANNEL_13

#define ADC_12V_A_RCU RCU_GPIOA
#define ADC_12V_A_PORT GPIOA
#define ADC_12V_A_PIN GPIO_PIN_4
#define ADC_12V_A_CHANNEL ADC_CHANNEL_4

#define MCU_TEMP ADC_CHANNEL_16

//RCU
static rcu_periph_enum adcGpioRcu[ADC_NUMS] = {
    ADC_12V_MXM_RCU,
    ADC_12V_CPU_RCU,
    ADC_3V3_CPU_RCU,
    ADC_5V_BOARD_RCU,
    ADC_3V3_BOARD_RCU,
    ADC_12V_A_RCU};
//PORT
static uint32_t adcPort[ADC_NUMS] = {
    ADC_12V_MXM_PORT,
    ADC_12V_CPU_PORT,
    ADC_3V3_CPU_PORT,
    ADC_5V_BOARD_PORT,
    ADC_3V3_BOARD_PORT,
    ADC_12V_A_PORT};
//PIN
static uint32_t adcPin[ADC_NUMS] = {
    ADC_12V_MXM_PIN,
    ADC_12V_CPU_PIN,
    ADC_3V3_CPU_PIN,
    ADC_5V_BOARD_PIN,
    ADC_3V3_BOARD_PIN,
    ADC_12V_A_PIN};
//ADC_Channel
static uint32_t adcChannel[ADC_NUMS] = {
    ADC_12V_MXM_CHANNEL,
    ADC_12V_CPU_CHANNEL,
    ADC_3V3_CPU_CHANNEL,
    ADC_5V_BOARD_CHANNEL,
    ADC_3V3_BOARD_CHANNEL,
    ADC_12V_A_CHANNEL};
//2023.12.01修改，硬件改动，修改电流采集算法10>>8.5
static float xNum[ADC_NUMS] = {11, 11, 3, 6.1, 3, 8.5};

typedef enum
{
    mxm_12V = 0,
    cpu_12V,
    cpu_3V3,
    board_5V,
    board_3V3,
    VPX_12V_A,
} ADC_M;

#define HK_ADC_BASE ADC0
#define HK_ADC_RCU RCU_ADC0

#define SAMPLE_NUM_MAX (20)
/* ADC reference voltage. The voltage of pin VREFH. */
#define ADC_VREF (3.300f)
/* ADC accuracy. */
#define ADC1_ACCURACY (1ul << 12) // 分辨率为12位

void hk_adc_init(void);
uint16_t hk_adc_filter(uint32_t adc_periph, uint8_t channel);

// void adc_sample_thread(void);
#endif /* __HK_ADC_H__ */
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
