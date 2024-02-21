/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   hk_wdg.c
 **
 ** \brief  wdg app
 **
 ** \author macy@schkzy.cn
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
//#include "ipmb/hk_ipmb.h"
#include "gd32f4xx_rcu.h"
#include "gd32f4xx.h"
#include "wdg/hk_wdg.h"
#include "task.h"



// https://blog.csdn.net/zrb2753/article/details/106086972

//0x05分频系数128,32kHZ/128=250hz,载入数为4095.超16秒不喂,重启!!!



static void wwdgt_task(void)
{
    IWDG_INIT()
    while (1)
    {
        vTaskDelay(5000);
        IWDG_FEED();
    }
}


//看门狗任务
void wwdgt_thread(uint8_t uxPriority)
{
    xTaskCreate((TaskFunction_t)wwdgt_task,"wwdgt_task",
                configMINIMAL_STACK_SIZE,NULL,uxPriority,NULL);
}
