/*!
    \file  main.c
    \brief key demo

    \version 2016-08-15, V1.0.0, firmware for GD32F4xx
    \version 2018-12-12, V2.0.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2018, GigaDevice Semiconductor Inc.

    All rights reserved.

    Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software without
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
*/

#include <stdio.h>
#include "gd32f4xx.h"
#include "gd32f450z_eval.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "pin.h"
#include "fmc/fmc.h"
#include "fatfs/fatfs_user.h"
#include "rtc/rtc.h"
#include "spi/gd25qxx.h"
#include "upgrade/bootload.h"
#include "i2c/hk_i2c.h"
#include "wdg/hk_wdg.h"
#include "ipmb/ipmb.h"
#include "hk_sensor/hk_sensor.h"
#include "ymodem/hk_ymodem.h"
#include "adc/hk_adc.h"
#include "comm/basecpld.h"

extern void vRegisterSampleCLICommands(void);
extern void vUARTCommandConsoleStart(UBaseType_t uxPriority);

void disable_rcu(void);
void board_poweron_delay(void); // CPU启动延时

// RTOS Function
static TaskHandle_t AppTask_handle = NULL; /* 创建应用任务处理句柄 */

extern IPMB_SLOT_TYPE slot; //槽位

/***********************************************************************
 * @ 函数名  ： AppTaskCreate
 * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
 * @ 参数    ： 无
 * @ 返回值  ： 无
 **********************************************************************/
static void AppTaskCreate(void)
{
    //进入临界区
    // taskENTER_CRITICAL();

    /* sys init */
    /* gpio init */
    gpio_init();    
    /* file system init */
    fatfs_init();
    /* spi init */
    spi_flash_init();

    /* power on*/
    board_poweron_delay();

    /* ipmb rsp task */	//notice
    ipmb_rsp_ipmb1_thread(3);
    ipmb_rsp_ipmb2_thread(3);

    /* read sensor task */
    read_sensor_thread(5);

    /* wwdgt task */
    wwdgt_thread(6);

    /* delete AppTask_handle */
    vTaskDelete(AppTask_handle);

    //退出临界区
    // taskEXIT_CRITICAL();
}

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
#ifdef _IAP_BOOT_USED_
    boot_start();
#endif /* _IAP_BOOT_USED_ */

#ifdef _IAP_APP_USED_
    if (IS_APP_RUN_BK)
        nvic_vector_table_set(NVIC_VECTTAB_FLASH, BOOT_APP_FLASH_BK_START_ADDR & 0xffffff);
    else
        nvic_vector_table_set(NVIC_VECTTAB_FLASH, BOOT_APP_FLASH_START_ADDR & 0xffffff);

    disable_rcu();

    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* Xshell task */
    vUARTCommandConsoleStart(4);

    /* command */
    vRegisterSampleCLICommands();

    BaseType_t xReturn = pdPASS; /* 定义一个创建信息返回值，默认为pdPASS */
    /* 创建AppTaskCreate任务 */
    xReturn = xTaskCreate((TaskFunction_t)AppTaskCreate, /* 任务入口函数 */
                          (const char *)"AppTaskCreate", /* 任务名字 */
                          (uint16_t)1024,                /* 任务栈大小 */
                          (void *)NULL,                  /* 任务入口函数参数 */
                          (UBaseType_t)5,                /* 任务的优先级 */
                          (TaskHandle_t *)NULL);         /* 任务控制块指针 */

    /* 启动任务调度 */
    if (pdPASS == xReturn)
        vTaskStartScheduler(); /* 启动任务，开启调度 */
    else
        printf("RTOS START FAIL \n");

#endif /* _IAP_APP_USED_ */

    while (1)
    {
    }
}

void disable_rcu(void)
{
    __set_PRIMASK(1);
    __set_PRIMASK(0);

    rcu_periph_clock_disable(RCU_DMA0);
    rcu_periph_clock_disable(RCU_DMA1);
    rcu_periph_clock_disable(RCU_USART0);
    rcu_periph_clock_disable(RCU_ENET);
    rcu_periph_clock_disable(RCU_ENETTX);
    rcu_periph_clock_disable(RCU_ENETRX);
    rcu_periph_clock_disable(RCU_GPIOA);
    rcu_periph_clock_disable(RCU_GPIOB);
    rcu_periph_clock_disable(RCU_GPIOC);
    rcu_periph_clock_disable(RCU_GPIOD);
    rcu_periph_clock_disable(RCU_GPIOG);
    rcu_periph_clock_disable(RCU_GPIOH);
    rcu_periph_clock_disable(RCU_GPIOI);
    rcu_periph_clock_disable(RCU_SYSCFG);
    rcu_periph_clock_disable(RCU_RTC);
    rcu_periph_clock_disable(RCU_PMU);
    rcu_periph_clock_disable(RCU_SDIO);
    rcu_periph_clock_disable(RCU_SPI0);
    rcu_periph_clock_disable(RCU_SPI1);
}


// 启动延时
void board_poweron_delay(void)
{
    uint8_t read_data[4] = {0};
    flash_read(FMC_START_DELAY_ADDRESS, 4, read_data);

    if (read_data[0] == 0)  //不开机
    {
        printf(" [power off]  start delay: 0 \r\n");
        return;
    }
    else if (read_data[0] < 40)
    {
        vTaskDelay(200 * read_data[0]);
    }
    else if (read_data[0] == 128)
    {
        vTaskDelay(200 * slot); //根据槽位进行延时
    }

    /* power on*/
    bc_cpu_power_on();
}
