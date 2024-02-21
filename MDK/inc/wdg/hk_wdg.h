/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file 
 **
 ** \brief  
 **
 ** \author macy@schkzy.cn
 **
 ******************************************************************************/
#ifndef __HK_WDG_H__
#define __HK_WDG_H__

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "gd32f4xx_wwdgt.h"


void wwdgt_init(void);
void wwdgt_thread(uint8_t uxPriority);


#define IWDG 0x40003000
#define IWDG_CTLR REG32((IWDG) + 0x00U)
#define IWDG_PSR REG32((IWDG) + 0x04U)
#define IWDG_RLDR REG32((IWDG) + 0x08U)
#define IWDG_STR REG32((IWDG) + 0x0CU)

#define IWDG_INIT()     \
    IWDG_CTLR = 0x5555; \
    IWDG_PSR |= 0x06;   \
    IWDG_CTLR = 0xCCCC;

#define IWDG_FEED()  IWDG_CTLR = 0xAAAA;

#endif 
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
