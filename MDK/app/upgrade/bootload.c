/*!
    \file  bootload.c
    \brief bootload file

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

#include "gd32f4xx.h"

#include <stdio.h>
#include <string.h>

#include "pin.h"
#include "upgrade/bootload.h"

static boot_state_enum _boot_branch_state_get(void)
{
    if(IS_APP_RUN)
        return BOOT_STATE_JUMP_TO_APP;
    
    if(IS_APP_RUN_BK)
        return BOOT_STATE_JUMP_TO_APP_BK;
    
    return BOOT_STATE_LOAD;
}

static void _boot_jump_to_app(uint32_t addr)
{
    p_function jump_to_application;
    uint32_t jumpaddress;
    
    /* check if valid stack address (RAM address) then jump to user application */
    /* RAM: 256K  0x2FFFFFFF - (0x40000 - 1) */
    if (0x20000000 == ((*(__IO uint32_t *)addr) & 0x2FFC0000))
    {
        /* jump to user application */
        jumpaddress = *(__IO uint32_t*) (addr + 4);
        jump_to_application = (p_function) jumpaddress;

        /* initialize user application's stack pointer */
        __set_MSP(*(__IO uint32_t*) addr);
        jump_to_application();
    }
}

//UART4
static void _boot_init_com(void)
{
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    /* enable UART clock */
    rcu_periph_clock_enable(RCU_UART4);

    /* connect port to UARTx_Tx */
    gpio_af_set(BMC_UART4_TX_PORT, GPIO_AF_8, BMC_UART4_TX_PIN);

    /* connect port to UARTx_Rx */
    gpio_af_set(BMC_UART4_RX_PORT, GPIO_AF_8, BMC_UART4_RX_PIN);

    /* configure UART Tx as alternate function push-pull */
    gpio_mode_set(BMC_UART4_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,BMC_UART4_TX_PIN);
    gpio_output_options_set(BMC_UART4_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,BMC_UART4_TX_PIN);

    /* configure UART Rx as alternate function push-pull */
    gpio_mode_set(BMC_UART4_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP,BMC_UART4_RX_PIN);
    gpio_output_options_set(BMC_UART4_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,BMC_UART4_RX_PIN);

    /* UART configure */
    usart_deinit(UART4);
    usart_baudrate_set(UART4, BAUDRATE_BOOT);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_enable(UART4);
}

void boot_start(void)
{
    _boot_init_com();

    switch (_boot_branch_state_get())
    {
    case BOOT_STATE_JUMP_TO_APP:
        printf("bootload jump to app \r\n");
        _boot_jump_to_app(BOOT_APP_FLASH_START_ADDR);
        break;

        case BOOT_STATE_JUMP_TO_APP_BK:
            printf("bootload jump to app bk\r\n");
            _boot_jump_to_app(BOOT_APP_FLASH_BK_START_ADDR);
            break;
        
        case BOOT_STATE_LOAD:
            printf("bootload jump to load\r\n");
            _boot_jump_to_app(BOOT_APP_FLASH_START_ADDR);
            break;

        default:
            break;
    }
}
