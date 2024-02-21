/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   basecpld.c
 **
 ** \brief  mcu spi to cpld API 
 **
 ** \author macy@schkzy.cn
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/

#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "gd32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "comm/basecpld.h"
#include "spi/gd25qxx.h"

uint8_t bc_get_version(void)
{
    return bc_read(BASE_CPLD_VERSION);
}

uint8_t bc_get_cpu_pwr_status(void)
{
    return bc_read(BASE_CPLD_CPU_PWR_STA);
}

uint8_t bc_get_cpu_power_en(void)
{
    return bc_read(BASE_CPLD_CPU_PWR_EN);
}

uint8_t bc_get_cpu_present_reset(void)
{
    return bc_read(BASE_CPLD_PRESENT_RST);
}

uint8_t bc_get_power_status(void)
{
    return bc_read(BASE_CPLD_PWR_STA);
}

uint8_t bc_get_power_ops(void)
{
    return bc_read(BASE_CPLD_PWR_OPS);
}

uint8_t bc_get_inlent_temp(void)
{
    return bc_read(BASE_CPLD_INLENT_TEMP);
}

uint8_t bc_get_outlent_temp(void)
{
    return bc_read(BASE_CPLD_OUTLENT_TEMP);
}

uint8_t bc_get_wave(void)
{
    return bc_read(BASE_CPLD_WAVE);
}

uint8_t bc_get_cpu2_msata_status(void)
{
    return bc_read(BASE_CPLD_2_MSATA_STA);
}

uint8_t bc_get_cpu1_msata_status(void)
{
    return bc_read(BASE_CPLD_1_MSATA_STA);
}

void bc_cpu_power_on(void)
{
    bc_write( BASE_CPLD_PWR_OPS, (bc_read(BASE_CPLD_PWR_OPS) | BIT(7) ) );
}

void bc_cpu_power_off(void)
{
    bc_write( BASE_CPLD_PWR_OPS, (bc_read(BASE_CPLD_PWR_OPS) & ~BIT(7)) );
}

void bc_heat_on(void)
{
    bc_write( BASE_CPLD_PWR_OPS, (bc_get_power_ops() | 0x20) );
}

void bc_heat_off(void)
{
    uint8_t tmp = bc_get_cpu_pwr_status() << 3;
    tmp |= (bc_get_cpu_pwr_status() >> 6) << 6;

    bc_write( BASE_CPLD_PWR_OPS, tmp );
}

uint8_t bc_read(uint8_t addr)
{
    uint8_t value = 0xff;

    value = spi_1_read(addr);

    // printf("SPI Address [0x%X]=[0x%X]\r\n", addr, value);

    return value;
}

uint8_t bc_write(uint8_t addr, uint8_t value)
{
    return spi_1_write(addr,value);
}


