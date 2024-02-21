/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   hk_adc.c
 **
 ** \brief  adc app
 **
 ** \author macy@schkzy.cn
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/

#include "stdint.h"
#include "stdio.h"
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "console/serial.h"
#include "adc/hk_adc.h"
/*!
    \brief      configure the GPIO peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hk_adc_gpio_config(void)
{
    /* enable GPIO clock */
    for (uint8_t i = 0; i < ADC_NUMS; i++)
    {
        rcu_periph_clock_enable(adcGpioRcu[i]);
    }
    adc_clock_config(ADC_ADCCK_PCLK2_DIV8);
    /* config the GPIO as analog mode */
    for (uint8_t i = 0; i < ADC_NUMS; i++)
        gpio_mode_set(adcPort[i], GPIO_MODE_ANALOG, GPIO_PUPD_NONE, adcPin[i]);
}

/*!
    \brief      configure the ADC peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void hk_adc_config(uint32_t adc_periph)
{

    /* ADC mode config */
    adc_sync_mode_config(ADC_SYNC_MODE_INDEPENDENT);
    /* ADC contineous function disable */
    adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, DISABLE);
    /* ADC scan mode disable */
    adc_special_function_config(adc_periph, ADC_SCAN_MODE, DISABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);
    /* ADC channel length config */
    adc_channel_length_config(adc_periph, ADC_REGULAR_CHANNEL, 1U);

    /* ADC trigger config */
    adc_external_trigger_source_config(adc_periph, ADC_REGULAR_CHANNEL, ADC_EXTTRIG_REGULAR_T0_CH0);
    adc_external_trigger_config(adc_periph, ADC_REGULAR_CHANNEL, EXTERNAL_TRIGGER_DISABLE);

    /* enable ADC interface */
    adc_enable(adc_periph);

    /* ADC calibration and reset calibration */
    adc_calibration_enable(adc_periph);
}

/*!
    \brief      ADC channel sample
    \param[in]  none
    \param[out] none
    \retval     none
*/
static uint16_t hk_adc_channel_sample(uint32_t adc_periph, uint8_t channel)
{
    /* ADC regular channel config */
    adc_regular_channel_config(adc_periph, 0U, channel, ADC_SAMPLETIME_15);
    /* ADC software trigger enable */
    adc_software_trigger_enable(adc_periph, ADC_REGULAR_CHANNEL);

    /* wait the end of conversion flag */
    while (!adc_flag_get(adc_periph, ADC_FLAG_EOC))
        ;
    /* clear the end of conversion flag */
    adc_flag_clear(adc_periph, ADC_FLAG_EOC);
    /* return regular channel sample value */
    return (adc_regular_data_read(adc_periph));
}

void hk_adc_init(void)
{
    /* reset ADC */
    adc_deinit();
    rcu_periph_clock_enable(HK_ADC_RCU);
    /* GPIO configuration */
    hk_adc_gpio_config();

    adc_channel_16_to_18(ADC_TEMP_VREF_CHANNEL_SWITCH, ENABLE);
    // /* ADC configuration */
    hk_adc_config(HK_ADC_BASE);

}

/**
 *******************************************************************************
 ** \brief  adc_filter——delete max and min element, then average
 **
 ******************************************************************************/
uint16_t hk_adc_filter(uint32_t adc_periph, uint8_t channel)
{
    volatile uint16_t count, temp, k, j;
    volatile uint32_t sum = 0;
    volatile uint8_t ret = 0;
    uint16_t value_buf[SAMPLE_NUM_MAX] = {0};
    for (count = 0; count < SAMPLE_NUM_MAX; count++)
    {
        value_buf[count] = hk_adc_channel_sample(adc_periph, channel);
        vTaskDelay(5);
    }

    for (j = 0; j < SAMPLE_NUM_MAX - 1; j++)
    {
        for (k = 0; k < SAMPLE_NUM_MAX - j; k++)
        {
            if (value_buf[k] > value_buf[k + 1])
            {
                temp = value_buf[k];
                value_buf[k] = value_buf[k + 1];
                value_buf[k + 1] = temp;
            }
        }
    }

    for (count = 1; count < SAMPLE_NUM_MAX - 1; count++)
        sum += value_buf[count];

    return sum / (SAMPLE_NUM_MAX - 3);
}
