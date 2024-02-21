/*!
    \file    sdcard.c
    \brief   MMC card driver

    \version 2022-05-26, V2.0.0, demo for GD32F4xx
*/

/*
    Copyright (c) 2022, GigaDevice Semiconductor Inc.

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

#include "emmc/emmc.h"
#include "gd32f4xx_sdio.h"
#include "gd32f4xx_dma.h"
#include <stddef.h>

/* card status of R1 definitions */
#define MMC_R1_OUT_OF_RANGE                  BIT(31)                   /* command's argument was out of the allowed range */
#define MMC_R1_ADDRESS_ERROR                 BIT(30)                   /* misaligned address which did not match the block length */
#define MMC_R1_BLOCK_LEN_ERROR               BIT(29)                   /* transferred block length is not allowed */
#define MMC_R1_ERASE_SEQ_ERROR               BIT(28)                   /* an error in the sequence of erase commands occurred */
#define MMC_R1_ERASE_PARAM                   BIT(27)                   /* an invalid selection of write-blocks for erase occurred */
#define MMC_R1_WP_VIOLATION                  BIT(26)                   /* the host attempts to write to a protected block or to the temporary or permanent write protected card */
#define MMC_R1_CARD_IS_LOCKED                BIT(25)                   /* the card is locked by the host */
#define MMC_R1_LOCK_UNLOCK_FAILED            BIT(24)                   /* a sequence or password error has been detected in lock/unlock card command */
#define MMC_R1_COM_CRC_ERROR                 BIT(23)                   /* CRC check of the previous command failed */
#define MMC_R1_ILLEGAL_COMMAND               BIT(22)                   /* command not legal for the card state */
#define MMC_R1_CARD_ECC_FAILED               BIT(21)                   /* card internal ECC was applied but failed to correct the data */
#define MMC_R1_CC_ERROR                      BIT(20)                   /* internal card controller error */
#define MMC_R1_GENERAL_UNKNOWN_ERROR         BIT(19)                   /* a general or an unknown error occurred during the operation */
#define MMC_R1_CSD_OVERWRITE                 BIT(16)                   /* read only section of the CSD does not match or attempt to reverse the copy or permanent WP bits */
#define MMC_R1_WP_ERASE_SKIP                 BIT(15)                   /* partial address space was erased */
#define MMC_R1_CARD_ECC_DISABLED             BIT(14)                   /* command has been executed without using the internal ECC */
#define MMC_R1_ERASE_RESET                   BIT(13)                   /* an erase sequence was cleared before executing */
#define MMC_R1_READY_FOR_DATA                BIT(8)                    /* correspond to buffer empty signaling on the bus */
#define MMC_R1_APP_CMD                       BIT(5)                    /* card will expect ACMD */
#define MMC_R1_AKE_SEQ_ERROR                 BIT(3)                    /* error in the sequence of the authentication process */
#define MMC_R1_ERROR_BITS                    (uint32_t)0xFDF9E008      /* all the R1 error bits */

/* card status of R6 definitions */
#define MMC_R6_COM_CRC_ERROR                 BIT(15)                   /* CRC check of the previous command failed */
#define MMC_R6_ILLEGAL_COMMAND               BIT(14)                   /* command not legal for the card state */
#define MMC_R6_GENERAL_UNKNOWN_ERROR         BIT(13)                   /* a general or an unknown error occurred during the operation */

/* card state */
#define MMC_CARDSTATE_IDLE                   ((uint8_t)0x00)           /* card is in idle state */
#define MMC_CARDSTATE_READY                  ((uint8_t)0x01)           /* card is in ready state */
#define MMC_CARDSTATE_IDENTIFICAT            ((uint8_t)0x02)           /* card is in identificat state */
#define MMC_CARDSTATE_STANDBY                ((uint8_t)0x03)           /* card is in standby state */
#define MMC_CARDSTATE_TRANSFER               ((uint8_t)0x04)           /* card is in transfer state */
#define MMC_CARDSTATE_DATA                   ((uint8_t)0x05)           /* card is in data sending state */
#define MMC_CARDSTATE_RECEIVING              ((uint8_t)0x06)           /* card is in receiving state */
#define MMC_CARDSTATE_PROGRAMMING            ((uint8_t)0x07)           /* card is in programming state */
#define MMC_CARDSTATE_DISCONNECT             ((uint8_t)0x08)           /* card is in disconnect state */
#define MMC_CARDSTATE_LOCKED                 ((uint32_t)0x02000000)    /* card is in locked state */

#define MMC_CHECK_PATTERN                    ((uint32_t)0x000001AA)    /* check pattern for CMD8 */
#define MMC_VOLTAGE_WINDOW                   ((uint32_t)0x80100000)    /* host 3.3V request in ACMD41 */

/* parameters for ACMD41(voltage validation) */
#define MMC_HIGH_CAPACITY                    ((uint32_t)0x40000000)    /* high capacity SD memory card */
#define MMC_STD_CAPACITY                     ((uint32_t)0x00000000)    /* standard capacity SD memory card */

/* SD bus width, check SCR register */
#define MMC_BUS_WIDTH_8BIT                   ((uint32_t)0x00140000)    /* 8-bit width bus mode */
#define MMC_BUS_WIDTH_4BIT                   ((uint32_t)0x00040000)    /* 4-bit width bus mode */
#define MMC_BUS_WIDTH_1BIT                   ((uint32_t)0x00010000)    /* 1-bit width bus mode */

/* masks for SCR register */
#define MMC_MASK_0_7BITS                     ((uint32_t)0x000000FF)    /* mask [7:0] bits */
#define MMC_MASK_8_15BITS                    ((uint32_t)0x0000FF00)    /* mask [15:8] bits */
#define MMC_MASK_16_23BITS                   ((uint32_t)0x00FF0000)    /* mask [23:16] bits */
#define MMC_MASK_24_31BITS                   ((uint32_t)0xFF000000)    /* mask [31:24] bits */

#define SDIO_FIFO_ADDR                      ((uint32_t)0x40012C80)    /* address of SDIO_FIFO */
#define MMC_FIFOHALF_WORDS                   ((uint32_t)0x00000008)    /* words of FIFO half full/empty */
#define MMC_FIFOHALF_BYTES                   ((uint32_t)0x00000020)    /* bytes of FIFO half full/empty */

#define MMC_DATATIMEOUT                      ((uint32_t)0xFFFFFFFF)    /* DSM data timeout */
#define MMC_MAX_VOLT_VALIDATION              ((uint32_t)0x0000FFFF)    /* the maximum times of voltage validation */
#define MMC_MAX_DATA_LENGTH                  ((uint32_t)0x01FFFFFF)    /* the maximum length of data */
#define MMC_ALLZERO                          ((uint32_t)0x00000000)    /* all zero */
#define MMC_RCA_SHIFT                        ((uint8_t)0x10)           /* RCA shift bits */
#define MMC_CLK_DIV_INIT                     ((uint16_t)0x0076)        /* SD clock division in initilization phase */
//#define MMC_CLK_DIV_TRANS                    ((uint16_t)0x0002)        /* SD clock division in transmission phase */
#define MMC_CLK_DIV_TRANS                    ((uint16_t)0x0020)        /* SD clock division in transmission phase */
#define MMC_CLK_DIV_TRANS_16M                 ((uint16_t)0x0001)       /* SD clock division in transmission phase */
#define MMC_CLK_DIV_TRANS_24M                 ((uint16_t)0x0000)       /* SD clock division in transmission phase */

#define SDIO_MASK_INTC_FLAGS                ((uint32_t)0x00C007FF)    /* mask flags of SDIO_INTC */

uint32_t sd_scr[2] = {0, 0};                                          /* content of SCR register */

static sdio_card_type_enum cardtype = SDIO_EMMC; /* EMMC */
static uint32_t sd_csd[4] = {0, 0, 0, 0};                             /* content of CSD register */
static uint32_t sd_cid[4] = {0, 0, 0, 0};                             /* content of CID register */
static uint16_t sd_rca = 0;                                           /* RCA of SD card */
static uint32_t transmode = MMC_POLLING_MODE;
static uint32_t totalnumber_bytes = 0, stopcondition = 0;
static __IO sd_error_enum transerror = MMC_OK;
static __IO uint32_t transend = 0, number_bytes = 0;

/* check if the command sent error occurs */
static sd_error_enum cmdsent_error_check(void);
/* check if error occurs for R1 response */
static sd_error_enum r1_error_check(uint8_t cmdindex);
/* check if error type for R1 response */
static sd_error_enum r1_error_type_check(uint32_t resp);
/* check if error occurs for R2 response */
static sd_error_enum r2_error_check(void);
/* check if error occurs for R3 response */
static sd_error_enum r3_error_check(void);
/* check if error occurs for R6 response */
static sd_error_enum r6_error_check(uint8_t cmdindex, uint16_t *prca);

/* get the state which the card is in */
static sd_error_enum sd_card_state_get(uint8_t *pcardstate);
/* configure the bus width mode */
static sd_error_enum sd_bus_width_config(uint32_t buswidth);
/* get the data block size */
static uint32_t sd_datablocksize_get(uint16_t bytesnumber);

/* configure the GPIO of SDIO interface */
static void gpio_config(void);
/* configure the RCU of SDIO and DMA */
static void rcu_config(void);
/* configure the DMA for SDIO transfer request */
static void dma_transfer_config(uint32_t *srcbuf, uint32_t bufsize);
/* configure the DMA for SDIO reveive request */
static void dma_receive_config(uint32_t *dstbuf, uint32_t bufsize);

void delay(uint32_t time)  //未进系统时调用
{
    uint32_t cnt;
    for(cnt = time; cnt > 0; --cnt) {
    }
}

//sd_card_info_struct sd_cardinfo;                            /* information of SD card */

/*!
    \brief      initialize the SD card and make it in standby state
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_init(void)
{
    sd_error_enum status = MMC_OK;
    /* configure the RCU and GPIO, deinitialize the SDIO */
    rcu_config();
    gpio_config();
    sdio_deinit();

    /* configure the clock and work voltage */
    status = sd_power_on();
    if(MMC_OK != status) {
        return status;
    }

    /* initialize the card and get CID and CSD of the card */
    status = sd_card_init();
    if(MMC_OK != status) {
        return status;
    }

    /* configure the SDIO peripheral */
    sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE, SDIO_CLOCKPWRSAVE_DISABLE, MMC_CLK_DIV_TRANS);
    sdio_bus_mode_set(SDIO_BUSMODE_1BIT);
    sdio_hardware_clock_disable();

    return status;
}

/*!
    \brief      initialize the card and get CID and CSD of the card
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_card_init(void)
{
    sd_error_enum status = MMC_OK;
    uint16_t temp_rca = 0x01;

    if(SDIO_POWER_OFF == sdio_power_state_get()) {
        status = MMC_OPERATION_IMPROPER;
        return status;
    }

    /* the card is not I/O only card */
    if(SDIO_EMMC == cardtype) {
        /* send CMD2(MMC_CMD_ALL_SEND_CID) to get the CID numbers */
        sdio_command_response_config(MMC_CMD_ALL_SEND_CID, (uint32_t)0x0, SDIO_RESPONSETYPE_LONG);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r2_error_check();

        if(MMC_OK != status) {
            return status;
        }

        /* store the CID numbers */
        sd_cid[0] = sdio_response_get(SDIO_RESPONSE0);
        sd_cid[1] = sdio_response_get(SDIO_RESPONSE1);
        sd_cid[2] = sdio_response_get(SDIO_RESPONSE2);
        sd_cid[3] = sdio_response_get(SDIO_RESPONSE3);

        /* send CMD3(SEND_RELATIVE_ADDR) to ask the card to publish a new relative address (RCA) */
        sdio_command_response_config(MMC_CMD_SEND_RELATIVE_ADDR, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r6_error_check(MMC_CMD_SEND_RELATIVE_ADDR, &temp_rca);
        if(MMC_OK != status) {
            return status;
        }

        /* the card is not I/O only card */
        sd_rca = temp_rca;

        /* send CMD9(SEND_CSD) to get the addressed card's card-specific data (CSD) */
        sdio_command_response_config(MMC_CMD_SEND_CSD, (uint32_t)(temp_rca << MMC_RCA_SHIFT), SDIO_RESPONSETYPE_LONG);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r2_error_check();
        if(MMC_OK != status) {
            return status;
        }

        /* store the card-specific data (CSD) */
        sd_csd[0] = sdio_response_get(SDIO_RESPONSE0);
        sd_csd[1] = sdio_response_get(SDIO_RESPONSE1);
        sd_csd[2] = sdio_response_get(SDIO_RESPONSE2);
        sd_csd[3] = sdio_response_get(SDIO_RESPONSE3);
    }

    return status;
}

/*!
    \brief      configure the clock and the work voltage, and get the card type
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_power_on(void)
{
    sd_error_enum status = MMC_OK;
    uint32_t response = 0;
    __IO uint32_t timeout = 0x4000;
    /* configure the SDIO peripheral */
    sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE, SDIO_CLOCKPWRSAVE_DISABLE, MMC_CLK_DIV_INIT);
    sdio_bus_mode_set(SDIO_BUSMODE_1BIT);
    sdio_hardware_clock_disable();
    sdio_power_state_set(SDIO_POWER_ON);

    delay(0x8FFF);
    delay(0x8FFF);

    /* enable SDIO_CLK clock output */
    sdio_clock_enable();

    /* send CMD0(GO_IDLE_STATE) to reset the card */
    sdio_command_response_config(MMC_CMD_GO_IDLE_STATE, (uint32_t)0x0, SDIO_RESPONSETYPE_NO);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);

    /* enable the CSM */
    sdio_csm_enable();
    delay(0x8FFF);

    /* check if command sent error occurs */
    status = cmdsent_error_check();
    if(MMC_OK != status) {
        return status;
    }

    cardtype = SDIO_EMMC;

    while(timeout--) {
        sdio_command_response_config(MMC_CMD_SEND_OP_COND, (uint32_t)0x40ff8000, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        /* enable the CSM */
        sdio_csm_enable();


        /* check if some error occurs */
        status = r3_error_check();
        if(MMC_OK != status) {
            return status;
        }
        if(!timeout) {
            status = MMC_ERROR;
            return status;
        }

        /* get the response and check card power up status bit(busy) */
        response = sdio_response_get(SDIO_RESPONSE0);

        if(0x80000000 == (response & 0x80000000)) {
            break;
        } else {
            continue;
        }
    }

    return status;
}

/*!
    \brief      close the power of SDIO
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_power_off(void)
{
    sd_error_enum status = MMC_OK;
    sdio_power_state_set(SDIO_POWER_OFF);
    return status;
}

/*!
    \brief      configure the bus mode
    \param[in]  busmode: the bus mode
      \arg        SDIO_BUSMODE_1BIT: 1-bit SDIO card bus mode
      \arg        SDIO_BUSMODE_4BIT: 4-bit SDIO card bus mode
      \arg        SDIO_BUSMODE_8BIT: 8-bit SDIO card bus mode (MMC only)
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_bus_mode_config(uint32_t busmode)
{
    sd_error_enum status = MMC_OK;

    if(SDIO_BUSMODE_8BIT == busmode) {
        /* send CMD6(MMC_CMD_SWITCH) to set the width */
        sdio_command_response_config(MMC_CMD_SWITCH_FUNC, (uint32_t)0x03B70200, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        /* enable the CSM */
        sdio_csm_enable();

        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SWITCH_FUNC);
        if(MMC_OK != status) {
            return status;
        }

        /* configure SD bus width and the SDIO */
        status = sd_bus_width_config(MMC_BUS_WIDTH_8BIT);
        if(MMC_OK == status) {
            sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE,
                              SDIO_CLOCKPWRSAVE_DISABLE, MMC_CLK_DIV_TRANS_24M);
            sdio_bus_mode_set(busmode);
            sdio_hardware_clock_enable();
        }
    } else if(SDIO_BUSMODE_4BIT == busmode) {
        /* send CMD6(MMC_CMD_SWITCH) to set the width */
        sdio_command_response_config(MMC_CMD_SWITCH_FUNC, (uint32_t)0x03B70100, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        /* enable the CSM */
        sdio_csm_enable();

        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SWITCH_FUNC);
        if(MMC_OK != status) {
            return status;
        }
        /* configure SD bus width and the SDIO */
        status = sd_bus_width_config(MMC_BUS_WIDTH_4BIT);
        if(MMC_OK == status) {
            sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE,
                              SDIO_CLOCKPWRSAVE_DISABLE, MMC_CLK_DIV_TRANS_16M);
            sdio_bus_mode_set(busmode);
            sdio_hardware_clock_enable();
        }
    } else if(SDIO_BUSMODE_1BIT == busmode) {
        /* configure SD bus width and the SDIO */
        status = sd_bus_width_config(MMC_BUS_WIDTH_1BIT);
        if(MMC_OK == status) {
            sdio_clock_config(SDIO_SDIOCLKEDGE_RISING, SDIO_CLOCKBYPASS_DISABLE,
                              SDIO_CLOCKPWRSAVE_DISABLE, MMC_CLK_DIV_TRANS_24M);
            sdio_bus_mode_set(busmode);
            sdio_hardware_clock_enable();
        }
    } else {
        status = MMC_PARAMETER_INVALID;
    }

    return status;
}

/*!
    \brief      configure the mode of transmission
    \param[in]  txmode: transfer mode
      \arg        MMC_DMA_MODE: DMA mode
      \arg        MMC_POLLING_MODE: polling mode
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_transfer_mode_config(uint32_t txmode)
{
    sd_error_enum status = MMC_OK;
    /* set the transfer mode */
    if((MMC_DMA_MODE == txmode) || (MMC_POLLING_MODE == txmode)) {
        transmode = txmode;
    } else {
        status = MMC_PARAMETER_INVALID;
    }
    return status;
}

/*!
    \brief      read a block data into a buffer from the specified address of a card
    \param[out] preadbuffer: a pointer that store a block read data
    \param[in]  readaddr: the read data address
    \param[in]  blocksize: the data block size
    \retval     sd_error_enum
*/
sd_error_enum sd_block_read(uint32_t *preadbuffer, uint32_t readaddr, uint16_t blocksize)
{
    /* initialize the variables */
    sd_error_enum status = MMC_OK;
    uint32_t count = 0, align = 0, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = preadbuffer;
    __IO uint32_t timeout = 0;
    uint32_t response = 0;

    if(NULL == preadbuffer) {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    transerror = MMC_OK;
    transend = 0;
    totalnumber_bytes = 0;
    /* clear all DSM configuration */
    sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_disable();
    sdio_dma_disable();

    /* check whether the card is locked */
    if(sdio_response_get(SDIO_RESPONSE0) & MMC_CARDSTATE_LOCKED) {
        status = MMC_LOCK_UNLOCK_FAILED;
        return status;
    }

    readaddr /= blocksize;

    /* send CMD13(SEND_STATUS), addressed card sends its status registers */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SEND_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO_RESPONSE0);
    timeout = 100000;
    
    while((0 == (response & MMC_R1_READY_FOR_DATA)) && (timeout > 0)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SEND_STATUS);
        if(MMC_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO_RESPONSE0);
    }
    if(0 == timeout) {
        return MMC_ERROR;
    }

    align = blocksize & (blocksize - 1);
    if((blocksize > 0) && (blocksize <= 2048) && (0 == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(MMC_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SET_BLOCKLEN);
        if(MMC_OK != status) {
            return status;
        }
    } else {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    stopcondition = 0;
    totalnumber_bytes = blocksize;

    /* configure SDIO data transmisson */
    sdio_data_config(MMC_DATATIMEOUT, totalnumber_bytes, datablksize);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOSDIO, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_enable();

    /* send CMD17(READ_SINGLE_BLOCK) to read a block */
    sdio_command_response_config(MMC_CMD_READ_SINGLE_BLOCK, (uint32_t)readaddr, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_READ_SINGLE_BLOCK);
    if(MMC_OK != status) {
        return status;
    }

    if(MMC_POLLING_MODE == transmode) {
        /* polling mode */
        while(!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
            if(RESET != sdio_flag_get(SDIO_FLAG_RFH)) {
                /* at least 8 words can be read in the FIFO */
                for(count = 0; count < MMC_FIFOHALF_WORDS; count++) {
                    *(ptempbuff + count) = sdio_data_read();
                    //printf("\n\r %x:  %x \n\r",count,*(ptempbuff + count));
                }
                ptempbuff += MMC_FIFOHALF_WORDS;
            }
        }
//        uint8_t *pdata = (uint8_t *)preadbuffer;

        /* whether some error occurs and return it */
        if(RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
            status = MMC_DATA_CRC_ERROR;
            sdio_flag_clear(SDIO_FLAG_DTCRCERR);
            return status;
        } else if(RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
            status = MMC_DATA_TIMEOUT;
            sdio_flag_clear(SDIO_FLAG_DTTMOUT);
            return status;
        } else if(RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
            status = MMC_RX_OVERRUN_ERROR;
            sdio_flag_clear(SDIO_FLAG_RXORE);
            return status;
        } else if(RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
            status = MMC_START_BIT_ERROR;
            sdio_flag_clear(SDIO_FLAG_STBITE);
            return status;
        }
        while(RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
            *ptempbuff = sdio_data_read();
            ++ptempbuff;
        }
        /* clear the SDIO_INTC flags */
        sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    } else if(MMC_DMA_MODE == transmode) {
        /* DMA mode */
        /* enable the SDIO corresponding interrupts and DMA function */
        sdio_interrupt_enable(SDIO_INT_CCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_RXORE | SDIO_INT_DTEND | SDIO_INT_STBITE);
        sdio_dma_enable();
        dma_receive_config(preadbuffer, blocksize);
        timeout = 100000;
        while((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_FLAG_FTF)) && (timeout > 0)) {
            timeout--;
            if(0 == timeout) {
                return MMC_ERROR;
            }
        }
    } else {
        status = MMC_PARAMETER_INVALID;
    }
    return status;
}

/*!
    \brief      read multiple blocks data into a buffer from the specified address of a card
    \param[out] preadbuffer: a pointer that store multiple blocks read data
    \param[in]  readaddr: the read data address
    \param[in]  blocksize: the data block size
    \param[in]  blocksnumber: number of blocks that will be read
    \retval     sd_error_enum
*/
sd_error_enum sd_multiblocks_read(uint32_t *preadbuffer, uint32_t readaddr, uint16_t blocksize, uint32_t blocksnumber)
{
    /* initialize the variables */
    sd_error_enum status = MMC_OK;
    uint32_t count = 0, align = 0, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = preadbuffer;
    __IO uint32_t timeout = 0;
    uint32_t response = 0;

    if(NULL == preadbuffer) {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    timeout = 100000;
    
    while((0 == (response & MMC_R1_READY_FOR_DATA)) && (timeout > 0)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SEND_STATUS);
        if(MMC_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO_RESPONSE0);
    }
    if(0 == timeout) {
        return MMC_ERROR;
    }

    transerror = MMC_OK;
    transend = 0;
    totalnumber_bytes = 0;
    /* clear all DSM configuration */
    sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_disable();
    sdio_dma_disable();

    /* check whether the card is locked */
    if(sdio_response_get(SDIO_RESPONSE0) & MMC_CARDSTATE_LOCKED) {
        status = MMC_LOCK_UNLOCK_FAILED;
        return status;
    }

    readaddr /= blocksize;

    align = blocksize & (blocksize - 1);
    if((blocksize > 0) && (blocksize <= 2048) && (0 == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(MMC_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SET_BLOCKLEN);
        if(MMC_OK != status) {
            return status;
        }
    } else {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    if(blocksnumber > 1) {
        if(blocksnumber * blocksize > MMC_MAX_DATA_LENGTH) {
            /* exceeds the maximum length */
            status = MMC_PARAMETER_INVALID;
            return status;
        }

        stopcondition = 1;
        totalnumber_bytes = blocksnumber * blocksize;

        /* configure the SDIO data transmisson */
        sdio_data_config(MMC_DATATIMEOUT, totalnumber_bytes, datablksize);
        sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOSDIO, SDIO_TRANSMODE_BLOCK);
        sdio_dsm_enable();

        /* send CMD18(READ_MULTIPLE_BLOCK) to read multiple blocks */
        sdio_command_response_config(MMC_CMD_READ_MULTIPLE_BLOCK, readaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_READ_MULTIPLE_BLOCK);
        if(MMC_OK != status) {
            return status;
        }

        if(MMC_POLLING_MODE == transmode) {
            /* polling mode */
            while(!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTEND | SDIO_FLAG_STBITE)) {
                if(RESET != sdio_flag_get(SDIO_FLAG_RFH)) {
                    /* at least 8 words can be read in the FIFO */
                    for(count = 0; count < MMC_FIFOHALF_WORDS; count++) {
                        *(ptempbuff + count) = sdio_data_read();
                    }
                    ptempbuff += MMC_FIFOHALF_WORDS;
                }
            }

            /* whether some error occurs and return it */
            if(RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
                status = MMC_DATA_CRC_ERROR;
                sdio_flag_clear(SDIO_FLAG_DTCRCERR);
                return status;
            } else if(RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
                status = MMC_DATA_TIMEOUT;
                sdio_flag_clear(SDIO_FLAG_DTTMOUT);
                return status;
            } else if(RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
                status = MMC_RX_OVERRUN_ERROR;
                sdio_flag_clear(SDIO_FLAG_RXORE);
                return status;
            } else if(RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
                status = MMC_START_BIT_ERROR;
                sdio_flag_clear(SDIO_FLAG_STBITE);
                return status;
            }
            while(RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
                *ptempbuff = sdio_data_read();
                ++ptempbuff;
            }

            if(RESET != sdio_flag_get(SDIO_FLAG_DTEND)) {
                /* send CMD12(STOP_TRANSMISSION) to stop transmission */
                sdio_command_response_config(MMC_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
                sdio_wait_type_set(SDIO_WAITTYPE_NO);
                sdio_csm_enable();
                /* check if some error occurs */
                status = r1_error_check(MMC_CMD_STOP_TRANSMISSION);
                if(MMC_OK != status) {
                    return status;
                }
            }
            sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
        } else if(MMC_DMA_MODE == transmode) {
            /* DMA mode */
            /* enable the SDIO corresponding interrupts and DMA function */
            sdio_interrupt_enable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_RXORE | SDIO_INT_DTEND | SDIO_INT_STBITE);
            sdio_dma_enable();
            dma_receive_config(preadbuffer, totalnumber_bytes);

            timeout = 0xFFFFFFFF;
            while((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_FLAG_FTF)) && (timeout > 0)) {
                timeout--;
                if(0 == timeout) {
                    return MMC_ERROR;
                }
            }
            while((0 == transend) && (MMC_OK == transerror)) {
            }
            
            /* send CMD12(STOP_TRANSMISSION) to stop transmission */
            sdio_command_response_config(MMC_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
            sdio_wait_type_set(SDIO_WAITTYPE_NO);
            sdio_csm_enable();
            /* check if some error occurs */
            status = r1_error_check(MMC_CMD_STOP_TRANSMISSION);
            if(MMC_OK != status) {
                return status;
            }
            if(MMC_OK != transerror) {
                return transerror;
            }
        } else {
            status = MMC_PARAMETER_INVALID;
        }
    }

    return status;
}


sd_error_enum sd_block_write(uint32_t *pwritebuffer, uint32_t writeaddr, uint16_t blocksize)
{
    /* initialize the variables */
    sd_error_enum status = MMC_OK;
    uint8_t cardstate = 0;
    uint32_t count = 0, align = 0, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = pwritebuffer;
    __IO uint32_t timeout = 0;
    uint32_t transbytes = 0, restwords = 0, response = 0;

    if(NULL == pwritebuffer) {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    transerror = MMC_OK;
    transend = 0;
    totalnumber_bytes = 0;
    /* clear all DSM configuration */
    sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_disable();
    sdio_dma_disable();

    /* check whether the card is locked */
    if(sdio_response_get(SDIO_RESPONSE0) & MMC_CARDSTATE_LOCKED) {
        status = MMC_LOCK_UNLOCK_FAILED;
        return status;
    }

    writeaddr /= blocksize;

    align = blocksize & (blocksize - 1);
    if((blocksize > 0) && (blocksize <= 2048) && (0 == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(MMC_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SET_BLOCKLEN);
        if(MMC_OK != status) {
            return status;
        }
    } else {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status registers */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SEND_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO_RESPONSE0);
    timeout = 100000;

    while((0 == (response & MMC_R1_READY_FOR_DATA)) && (timeout > 0)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SEND_STATUS);
        if(MMC_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO_RESPONSE0);
    }
    if(0 == timeout) {
        return MMC_ERROR;
    }

    /* send CMD24(WRITE_BLOCK) to write a block */
    sdio_command_response_config(MMC_CMD_WRITE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_WRITE_BLOCK);
    if(MMC_OK != status) {
        return status;
    }

    stopcondition = 0;
    totalnumber_bytes = blocksize;

    /* configure the SDIO data transmisson */
    sdio_data_config(MMC_DATATIMEOUT, totalnumber_bytes, datablksize);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_enable();

    /* send CMD13(SEND_STATUS), addressed card sends its status registers */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SEND_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO_RESPONSE0);
    timeout = 100000;



    while((0 == (response & MMC_R1_READY_FOR_DATA)) && (timeout > 0)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SEND_STATUS);
        if(MMC_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO_RESPONSE0);
    }
    if(0 == timeout) {
        return MMC_ERROR;
    }

    if(MMC_POLLING_MODE == transmode) {
        /* polling mode */
        while(!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_TXURE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
            if(RESET != sdio_flag_get(SDIO_FLAG_TFH)) {
                /* at least 8 words can be written into the FIFO */
                if((totalnumber_bytes - transbytes) < MMC_FIFOHALF_BYTES) {
                    restwords = (totalnumber_bytes - transbytes) / 4 + (((totalnumber_bytes - transbytes) % 4 == 0) ? 0 : 1);
                    for(count = 0; count < restwords; count++) {
                        sdio_data_write(*ptempbuff);
                        ++ptempbuff;
                        transbytes += 4;
                    }
                } else {
                    for(count = 0; count < MMC_FIFOHALF_WORDS; count++) {
                        sdio_data_write(*(ptempbuff + count));
                    }
                    /* 8 words(32 bytes) has been transferred */
                    ptempbuff += MMC_FIFOHALF_WORDS;
                    transbytes += MMC_FIFOHALF_BYTES;

                    //printf("\r\n %x %x\r\n",ptempbuff,transbytes);



                }
            }
        }
        
        /* whether some error occurs and return it */
        if(RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
            status = MMC_DATA_CRC_ERROR;
            sdio_flag_clear(SDIO_FLAG_DTCRCERR);
            return status;
        } else if(RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
            status = MMC_DATA_TIMEOUT;
            sdio_flag_clear(SDIO_FLAG_DTTMOUT);
            return status;
        } else if(RESET != sdio_flag_get(SDIO_FLAG_TXURE)) {
            status = MMC_TX_UNDERRUN_ERROR;
            sdio_flag_clear(SDIO_FLAG_TXURE);
            return status;
        } else if(RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
            status = MMC_START_BIT_ERROR;
            sdio_flag_clear(SDIO_FLAG_STBITE);
            return status;
        }
    } else if(MMC_DMA_MODE == transmode) {
        /* DMA mode */
        /* enable the SDIO corresponding interrupts and DMA */
        sdio_interrupt_enable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_DTEND | SDIO_INT_STBITE);
        dma_transfer_config(pwritebuffer, blocksize);
        sdio_dma_enable();

        timeout = 100000;
        while((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_INTF_FTFIF)) && (timeout > 0)) {
            timeout--;
            if(0 == timeout) {
                return MMC_ERROR;
            }
        }
        while((0 == transend) && (MMC_OK == transerror)) {
        }

        if(MMC_OK != transerror) {
            return transerror;
        }
    } else {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    /* clear the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((MMC_OK == status) && ((MMC_CARDSTATE_PROGRAMMING == cardstate) || (MMC_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      write multiple blocks data to the specified address of a card
    \param[in]  pwritebuffer: a pointer that store multiple blocks data to be transferred
    \param[in]  writeaddr: the read data address
    \param[in]  blocksize: the data block size
    \param[in]  blocksnumber: number of blocks that will be written
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_multiblocks_write(uint32_t *pwritebuffer, uint32_t writeaddr, uint16_t blocksize, uint32_t blocksnumber)
{
    /* initialize the variables */
    sd_error_enum status = MMC_OK;
    uint8_t cardstate = 0;
    uint32_t count = 0, align = 0, datablksize = SDIO_DATABLOCKSIZE_1BYTE, *ptempbuff = pwritebuffer;
    uint32_t transbytes = 0, restwords = 0;
    __IO uint32_t timeout = 0;

    if(NULL == pwritebuffer) {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    transerror = MMC_OK;
    transend = 0;
    totalnumber_bytes = 0;
    /* clear all DSM configuration */
    sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_disable();
    sdio_dma_disable();

    /* check whether the card is locked */
    if(sdio_response_get(SDIO_RESPONSE0) & MMC_CARDSTATE_LOCKED) {
        status = MMC_LOCK_UNLOCK_FAILED;
        return status;
    }

    writeaddr /= blocksize;

    align = blocksize & (blocksize - 1);
    if((blocksize > 0) && (blocksize <= 2048) && (0 == align)) {
        datablksize = sd_datablocksize_get(blocksize);
        /* send CMD16(SET_BLOCKLEN) to set the block length */
        sdio_command_response_config(MMC_CMD_SET_BLOCKLEN, (uint32_t)blocksize, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();

        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SET_BLOCKLEN);
        if(MMC_OK != status) {
            return status;
        }
    } else {
        status = MMC_PARAMETER_INVALID;
        return status;
    }


    /* send CMD13(SEND_STATUS), addressed card sends its status registers */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SEND_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    if(blocksnumber > 1) {
        if(blocksnumber * blocksize > MMC_MAX_DATA_LENGTH) {
            status = MMC_PARAMETER_INVALID;
            return status;
        }

        /* send ACMD23(SET_WR_BLK_ERASE_COUNT) to set the number of write blocks to be preerased before writing */
        sdio_command_response_config(MMC_APPCMD_SET_WR_BLK_ERASE_COUNT, blocksnumber, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_APPCMD_SET_WR_BLK_ERASE_COUNT);
        if(MMC_OK != status) {
            return status;
        }

        /* send CMD25(WRITE_MULTIPLE_BLOCK) to continuously write blocks of data */
        sdio_command_response_config(MMC_CMD_WRITE_MULTIPLE_BLOCK, writeaddr, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_WRITE_MULTIPLE_BLOCK);
        if(MMC_OK != status) {
            return status;
        }

        stopcondition = 1;
        totalnumber_bytes = blocksnumber * blocksize;

        /* configure the SDIO data transmisson */
        sdio_data_config(MMC_DATATIMEOUT, totalnumber_bytes, datablksize);
        sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
        sdio_dsm_enable();

        if(MMC_POLLING_MODE == transmode) {
            /* polling mode */
            while(!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_TXURE | SDIO_FLAG_DTEND | SDIO_FLAG_STBITE)) {
                if(RESET != sdio_flag_get(SDIO_FLAG_TFH)) {
                    /* at least 8 words can be written into the FIFO */
                    if(!((totalnumber_bytes - transbytes) < MMC_FIFOHALF_BYTES)) {
                        for(count = 0; count < MMC_FIFOHALF_WORDS; count++) {
                            sdio_data_write(*(ptempbuff + count));
                        }
                        /* 8 words(32 bytes) has been transferred */
                        ptempbuff += MMC_FIFOHALF_WORDS;
                        transbytes += MMC_FIFOHALF_BYTES;
                    } else {
                        restwords = (totalnumber_bytes - transbytes) / 4 + (((totalnumber_bytes - transbytes) % 4 == 0) ? 0 : 1);
                        for(count = 0; count < restwords; count++) {
                            sdio_data_write(*ptempbuff);
                            ++ptempbuff;
                            transbytes += 4;
                        }
                    }
                }
            }

            /* whether some error occurs and return it */
            if(RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
                status = MMC_DATA_CRC_ERROR;
                sdio_flag_clear(SDIO_FLAG_DTCRCERR);
                return status;
            } else if(RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
                status = MMC_DATA_TIMEOUT;
                sdio_flag_clear(SDIO_FLAG_DTTMOUT);
                return status;
            } else if(RESET != sdio_flag_get(SDIO_FLAG_TXURE)) {
                status = MMC_TX_UNDERRUN_ERROR;
                sdio_flag_clear(SDIO_FLAG_TXURE);
                return status;
            } else if(RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
                status = MMC_START_BIT_ERROR;
                sdio_flag_clear(SDIO_FLAG_STBITE);
                return status;
            }

            if(RESET != sdio_flag_get(SDIO_FLAG_DTEND)) {
                if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype) ||
                        (SDIO_HIGH_CAPACITY_SD_CARD == cardtype)) {
                    /* send CMD12(STOP_TRANSMISSION) to stop transmission */
                    sdio_command_response_config(MMC_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
                    sdio_wait_type_set(SDIO_WAITTYPE_NO);
                    sdio_csm_enable();
                    /* check if some error occurs */
                    status = r1_error_check(MMC_CMD_STOP_TRANSMISSION);
                    if(MMC_OK != status) {
                        return status;
                    }
                }
            }
            sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
        } else if(MMC_DMA_MODE == transmode) {
            /* DMA mode */
            /* enable SDIO corresponding interrupts and DMA */
            sdio_interrupt_enable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_TXURE | SDIO_INT_DTEND | SDIO_INT_STBITE);
            sdio_dma_enable();
            dma_transfer_config(pwritebuffer, totalnumber_bytes);

            timeout = 200000;
            while((RESET == dma_flag_get(DMA1, DMA_CH3, DMA_INTF_FTFIF) && (timeout > 0))) {
                timeout--;
                if(0 == timeout) {
                    return MMC_ERROR;
                }
            }
            while((0 == transend) && (MMC_OK == transerror)) {
            }
            if(MMC_OK != transerror) {
                return transerror;
            }
        } else {
            status = MMC_PARAMETER_INVALID;
            return status;
        }
    }

    /* clear the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((MMC_OK == status) && ((MMC_CARDSTATE_PROGRAMMING == cardstate) || (MMC_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      erase a continuous area of a card
    \param[in]  startaddr: the start address
    \param[in]  endaddr: the end address
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_erase(uint32_t startaddr, uint32_t endaddr)
{
    /* initialize the variables */
    sd_error_enum status = MMC_OK;
    uint32_t count = 0;
    __IO uint32_t delay = 0;
    uint8_t cardstate = 0;

    /* send CMD35(ERASE_WR_BLK_START) to set the address of the first write block to be erased */
    sdio_command_response_config(MMC_CMD_ERASE_GRP_START, startaddr, SDIO_RESPONSETYPE_SHORT);
    //sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_ERASE_GRP_START);

    if(MMC_OK != status) {
        return status;
    }

    /* send CMD36(ERASE_WR_BLK_END) to set the address of the last write block of the continuous range to be erased */
    sdio_command_response_config(MMC_CMD_ERASE_GRP_END, endaddr, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_ERASE_GRP_END);
    if(MMC_OK != status) {
        return status;
    }


    /* send CMD38(ERASE) to set the address of the first write block to be erased */
    sdio_command_response_config(MMC_CMD_ERASE, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_ERASE);
    if(MMC_OK != status) {
        return status;
    }
    /* loop until the counter is reach to the calculated time */
    for(count = 0; count < delay; count++) {
    }
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((MMC_OK == status) && ((MMC_CARDSTATE_PROGRAMMING == cardstate) || (MMC_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      process all the interrupts which the corresponding flags are set
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_interrupts_process(void)
{
    transerror = MMC_OK;
    if(RESET != sdio_interrupt_flag_get(SDIO_INT_DTEND)) {
        sdio_interrupt_flag_clear(SDIO_INT_DTEND);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        transend = 1;
        number_bytes = 0;
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO_INT_DTCRCERR)) {
        sdio_interrupt_flag_clear(SDIO_INT_DTCRCERR);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        number_bytes = 0;
        transerror = MMC_DATA_CRC_ERROR;
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO_INT_DTTMOUT)) {
        sdio_interrupt_flag_clear(SDIO_INT_DTTMOUT);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        number_bytes = 0;
        transerror = MMC_DATA_TIMEOUT;
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO_INT_STBITE)) {
        sdio_interrupt_flag_clear(SDIO_INT_STBITE);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        number_bytes = 0;
        transerror = MMC_START_BIT_ERROR;
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO_INT_TXURE)) {
        sdio_interrupt_flag_clear(SDIO_INT_TXURE);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        number_bytes = 0;
        transerror = MMC_TX_UNDERRUN_ERROR;
        return transerror;
    }

    if(RESET != sdio_interrupt_flag_get(SDIO_INT_RXORE)) {
        sdio_interrupt_flag_clear(SDIO_INT_RXORE);
        /* disable all the interrupts */
        sdio_interrupt_disable(SDIO_INT_DTCRCERR | SDIO_INT_DTTMOUT | SDIO_INT_DTEND | SDIO_INT_STBITE |
                               SDIO_INT_TFH | SDIO_INT_RFH | SDIO_INT_TXURE | SDIO_INT_RXORE);
        number_bytes = 0;
        transerror = MMC_RX_OVERRUN_ERROR;
        return transerror;
    }
    return transerror;
}

/*!
    \brief      select or deselect a card
    \param[in]  cardrca: the RCA of a card
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_card_select_deselect(uint16_t cardrca)
{
    sd_error_enum status = MMC_OK;
    /* send CMD7(SELECT/DESELECT_CARD) to select or deselect the card */
    sdio_command_response_config(MMC_CMD_SELECT_DESELECT_CARD, (uint32_t)(cardrca << MMC_RCA_SHIFT), SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();

    status = r1_error_check(MMC_CMD_SELECT_DESELECT_CARD);
    return status;
}

/*!
    \brief      get the card status
    \param[in]  none
    \param[out] pcardstatus: a pointer that store card status
    \retval     sd_error_enum
*/
sd_error_enum sd_cardstatus_get(uint32_t *pcardstatus)
{
    sd_error_enum status = MMC_OK;
    if(NULL == pcardstatus) {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status register */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SEND_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    *pcardstatus = sdio_response_get(SDIO_RESPONSE0);
    return status;
}

/*!
    \brief      get the SD card status
    \param[in]  none
    \param[out] psdstatus: a pointer that store SD card status
    \retval     sd_error_enum
*/
sd_error_enum sd_sdstatus_get(uint32_t *psdstatus)
{
    sd_error_enum status = MMC_OK;
    uint32_t count = 0;

    /* check whether the card is locked */
    if(sdio_response_get(SDIO_RESPONSE0) & MMC_CARDSTATE_LOCKED) {
        status = MMC_LOCK_UNLOCK_FAILED;
        return(status);
    }

    /* send CMD16(SET_BLOCKLEN) to set the block length */
    sdio_command_response_config(MMC_CMD_SET_BLOCKLEN, (uint32_t)64, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SET_BLOCKLEN);
    if(MMC_OK != status) {
        return status;
    }

    /* send CMD55(APP_CMD) to indicate next command is application specific command */
    sdio_command_response_config(MMC_CMD_APP_CMD, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_APP_CMD);
    if(MMC_OK != status) {
        return status;
    }

    /* configure the SDIO data transmisson */
    sdio_data_config(MMC_DATATIMEOUT, (uint32_t)64, SDIO_DATABLOCKSIZE_64BYTES);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOSDIO, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_enable();

    /* send ACMD13(MMC_STATUS) to get the SD status */
    sdio_command_response_config(MMC_APPCMD_SD_STATUS, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_APPCMD_SD_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    while(!sdio_flag_get(SDIO_FLAG_DTCRCERR | SDIO_FLAG_DTTMOUT | SDIO_FLAG_RXORE | SDIO_FLAG_DTBLKEND | SDIO_FLAG_STBITE)) {
        if(RESET != sdio_flag_get(SDIO_FLAG_RFH)) {
            for(count = 0; count < MMC_FIFOHALF_WORDS; count++) {
                *(psdstatus + count) = sdio_data_read();
            }
            psdstatus += MMC_FIFOHALF_WORDS;
        }
    }

    /* whether some error occurs and return it */
    if(RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
        status = MMC_DATA_CRC_ERROR;
        sdio_flag_clear(SDIO_FLAG_DTCRCERR);
        return status;
    } else if(RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
        status = MMC_DATA_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_DTTMOUT);
        return status;
    } else if(RESET != sdio_flag_get(SDIO_FLAG_RXORE)) {
        status = MMC_RX_OVERRUN_ERROR;
        sdio_flag_clear(SDIO_FLAG_RXORE);
        return status;
    } else if(RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
        status = MMC_START_BIT_ERROR;
        sdio_flag_clear(SDIO_FLAG_STBITE);
        return status;
    }
    while(RESET != sdio_flag_get(SDIO_FLAG_RXDTVAL)) {
        *psdstatus = sdio_data_read();
        ++psdstatus;
    }

    /* clear the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    psdstatus -= 16;
    for(count = 0; count < 16; count++) {
        psdstatus[count] = ((psdstatus[count] & MMC_MASK_0_7BITS) << 24) | ((psdstatus[count] & MMC_MASK_8_15BITS) << 8) |
                           ((psdstatus[count] & MMC_MASK_16_23BITS) >> 8) | ((psdstatus[count] & MMC_MASK_24_31BITS) >> 24);
    }
    return status;
}

/*!
    \brief      stop an ongoing data transfer
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_transfer_stop(void)
{
    sd_error_enum status = MMC_OK;
    /* send CMD12(STOP_TRANSMISSION) to stop transmission */
    sdio_command_response_config(MMC_CMD_STOP_TRANSMISSION, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_STOP_TRANSMISSION);
    return status;
}

/*!
    \brief      lock or unlock a card
    \param[in]  lockstate: the lock state
      \arg        MMC_LOCK: lock the SD card
      \arg        MMC_UNLOCK: unlock the SD card
    \param[out] none
    \retval     sd_error_enum
*/
sd_error_enum sd_lock_unlock(uint8_t lockstate)
{
    sd_error_enum status = MMC_OK;
    uint8_t cardstate = 0, tempbyte = 0;
    uint32_t pwd1 = 0, pwd2 = 0, response = 0, timeout = 0;
    uint16_t tempccc = 0;

    /* get the card command classes from CSD */
    tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_24_31BITS) >> 24);
    tempccc = (uint16_t)((uint16_t)tempbyte << 4);
    tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_16_23BITS) >> 16);
    tempccc |= (uint16_t)((uint16_t)(tempbyte & 0xF0) >> 4);

    if(0 == (tempccc & MMC_CCC_LOCK_CARD)) {
        /* don't support the lock command */
        status = MMC_FUNCTION_UNSUPPORTED;
        return status;
    }
    /* password pattern */
    pwd1 = (0x01020600 | lockstate);
    pwd2 = 0x03040506;

    /* clear all DSM configuration */
    sdio_data_config(0, 0, SDIO_DATABLOCKSIZE_1BYTE);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_disable();
    sdio_dma_disable();

    /* send CMD16(SET_BLOCKLEN) to set the block length */
    sdio_command_response_config(MMC_CMD_SET_BLOCKLEN, (uint32_t)8, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SET_BLOCKLEN);
    if(MMC_OK != status) {
        return status;
    }

    /* send CMD13(SEND_STATUS), addressed card sends its status register */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_SEND_STATUS);
    if(MMC_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO_RESPONSE0);
    timeout = 100000;
    while((0 == (response & MMC_R1_READY_FOR_DATA)) && (timeout > 0)) {
        /* continue to send CMD13 to polling the state of card until buffer empty or timeout */
        --timeout;
        /* send CMD13(SEND_STATUS), addressed card sends its status registers */
        sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
        sdio_wait_type_set(SDIO_WAITTYPE_NO);
        sdio_csm_enable();
        /* check if some error occurs */
        status = r1_error_check(MMC_CMD_SEND_STATUS);
        if(MMC_OK != status) {
            return status;
        }
        response = sdio_response_get(SDIO_RESPONSE0);
    }
    if(0 == timeout) {
        return MMC_ERROR;
    }

    /* send CMD42(LOCK_UNLOCK) to set/reset the password or lock/unlock the card */
    sdio_command_response_config(MMC_CMD_LOCK_UNLOCK, (uint32_t)0x0, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();
    /* check if some error occurs */
    status = r1_error_check(MMC_CMD_LOCK_UNLOCK);
    if(MMC_OK != status) {
        return status;
    }

    response = sdio_response_get(SDIO_RESPONSE0);

    /* configure the SDIO data transmisson */
    sdio_data_config(MMC_DATATIMEOUT, (uint32_t)8, SDIO_DATABLOCKSIZE_8BYTES);
    sdio_data_transfer_config(SDIO_TRANSDIRECTION_TOCARD, SDIO_TRANSMODE_BLOCK);
    sdio_dsm_enable();

    /* write password pattern */
    sdio_data_write(pwd1);
    sdio_data_write(pwd2);

    /* whether some error occurs and return it */
    if(RESET != sdio_flag_get(SDIO_FLAG_DTCRCERR)) {
        status = MMC_DATA_CRC_ERROR;
        sdio_flag_clear(SDIO_FLAG_DTCRCERR);
        return status;
    } else if(RESET != sdio_flag_get(SDIO_FLAG_DTTMOUT)) {
        status = MMC_DATA_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_DTTMOUT);
        return status;
    } else if(RESET != sdio_flag_get(SDIO_FLAG_TXURE)) {
        status = MMC_TX_UNDERRUN_ERROR;
        sdio_flag_clear(SDIO_FLAG_TXURE);
        return status;
    } else if(RESET != sdio_flag_get(SDIO_FLAG_STBITE)) {
        status = MMC_START_BIT_ERROR;
        sdio_flag_clear(SDIO_FLAG_STBITE);
        return status;
    }

    /* clear the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    /* get the card state and wait the card is out of programming and receiving state */
    status = sd_card_state_get(&cardstate);
    while((MMC_OK == status) && ((MMC_CARDSTATE_PROGRAMMING == cardstate) || (MMC_CARDSTATE_RECEIVING == cardstate))) {
        status = sd_card_state_get(&cardstate);
    }
    return status;
}

/*!
    \brief      get the data transfer state
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
sd_transfer_state_enum sd_transfer_state_get(void)
{
    sd_transfer_state_enum transtate = MMC_NO_TRANSFER;
    if(RESET != sdio_flag_get(SDIO_FLAG_TXRUN | SDIO_FLAG_RXRUN)) {
        transtate = MMC_TRANSFER_IN_PROGRESS;
    }
    return transtate;
}

/*!
    \brief      get SD card capacity
    \param[in]  none
    \param[out] none
    \retval     capacity of the card(KB)
*/
uint64_t sd_card_capacity_get(void)
{
    uint8_t tempbyte = 0, devicesize_mult = 0, readblklen = 0;
    uint64_t capacity = 0, devicesize = 0;
    if((SDIO_STD_CAPACITY_SD_CARD_V1_1 == cardtype) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == cardtype)) {
        /* calculate the c_size(device size) */
        tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_8_15BITS) >> 8);
        devicesize = (uint32_t)((uint32_t)(tempbyte & 0x03) << 10);
        tempbyte = (uint8_t)(sd_csd[1] & MMC_MASK_0_7BITS);
        devicesize |= (uint32_t)((uint32_t)tempbyte << 2);
        tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_24_31BITS) >> 24);
        devicesize |= (uint32_t)((uint32_t)(tempbyte & 0xC0) >> 6);

        /* calculate the c_size_mult(device size multiplier) */
        tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_16_23BITS) >> 16);
        devicesize_mult = (tempbyte & 0x03) << 1;
        tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_8_15BITS) >> 8);
        devicesize_mult |= (tempbyte & 0x80) >> 7;

        /* calculate the read_bl_len */
        tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_16_23BITS) >> 16);
        readblklen = tempbyte & 0x0F;

        /* capacity = BLOCKNR*BLOCK_LEN, BLOCKNR = (C_SIZE+1)*MULT, MULT = 2^(C_SIZE_MULT+2), BLOCK_LEN = 2^READ_BL_LEN */
        capacity = (devicesize + 1) * (1 << (devicesize_mult + 2));
        capacity *= (1 << readblklen);

        /* change the unit of capacity to KByte */
        capacity /= 1024;
    } else if(SDIO_HIGH_CAPACITY_SD_CARD == cardtype) {
        /* calculate the c_size */
        tempbyte = (uint8_t)(sd_csd[1] & MMC_MASK_0_7BITS);
        devicesize = (uint32_t)((uint32_t)(tempbyte & 0x3F) << 16);
        tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_24_31BITS) >> 24);
        devicesize |= (uint32_t)((uint32_t)tempbyte << 8);
        tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_16_23BITS) >> 16);
        devicesize |= (uint32_t)tempbyte;

        /* capacity = (c_size+1)*512KByte */
        capacity = (devicesize + 1) * 512 * 1024;
    } else if (SDIO_EMMC == cardtype)
    {
        // tempbyte = (uint8_t)(sd_csd[1] >> 30);
        // devicesize = (uint8_t)( (sd_csd[1] & 0x3FF) << 2);
        // devicesize |= tempbyte;
        // printf("devicesize=222[%d]\r\n", devicesize);
        capacity = 2 * 512 * 1024;
    }

    return capacity;
}

/*!
    \brief      get the detailed information of the SD card based on received CID and CSD
    \param[in]  none
    \param[out] pcardinfo: a pointer that store the detailed card information
    \retval     sd_error_enum
*/
sd_error_enum sd_card_information_get(sd_card_info_struct *pcardinfo)
{
    sd_error_enum status = MMC_OK;
    uint8_t tempbyte = 0;

    if(NULL == pcardinfo) {
        status = MMC_PARAMETER_INVALID;
        return status;
    }

    /* store the card type and RCA */
    pcardinfo->card_type = cardtype;
    pcardinfo->card_rca = sd_rca;

    /* CID byte 0 */
    tempbyte = (uint8_t)((sd_cid[0] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_cid.mid = tempbyte;

    /* CID byte 1 */
    tempbyte = (uint8_t)((sd_cid[0] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_cid.cbx = tempbyte & 0x03;

    /* CID byte 2 */
    tempbyte = (uint8_t)((sd_cid[0] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_cid.oid = (uint16_t)tempbyte;

    /* CID byte 3 */
    tempbyte = (uint8_t)(sd_cid[0] & MMC_MASK_0_7BITS);
    pcardinfo->card_cid.pnm0 = (uint32_t)((uint32_t)tempbyte << 24);

    /* CID byte 4 */
    tempbyte = (uint8_t)((sd_cid[1] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_cid.pnm0 |= (uint32_t)((uint32_t)tempbyte << 16);

    /* CID byte 5 */
    tempbyte = (uint8_t)((sd_cid[1] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_cid.pnm0 |= (uint32_t)((uint32_t)tempbyte << 8);

    /* CID byte 6 */
    tempbyte = (uint8_t)((sd_cid[1] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_cid.pnm0 |= (uint32_t)(tempbyte);

    /* CID byte 7 */
    tempbyte = (uint8_t)(sd_cid[1] & MMC_MASK_0_7BITS);
    pcardinfo->card_cid.pnm1 = (uint16_t)((uint16_t)tempbyte << 8);

    /* CID byte 8 */
    tempbyte = (uint8_t)((sd_cid[2] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_cid.pnm1 |= (uint16_t)(tempbyte);

    /* CID byte 9 */
    tempbyte = (uint8_t)((sd_cid[2] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_cid.prv = tempbyte;

    /* CID byte 10 */
    tempbyte = (uint8_t)((sd_cid[2] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_cid.psn = (uint32_t)((uint32_t)tempbyte << 24);


    /* CID byte 11 */
    tempbyte = (uint8_t)(sd_cid[2] & MMC_MASK_0_7BITS);
    pcardinfo->card_cid.psn |= (uint32_t)((uint32_t)tempbyte << 16);

    /* CID byte 12 */
    tempbyte = (uint8_t)((sd_cid[3] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_cid.psn |= (uint32_t)((uint32_t)tempbyte << 8);

    /* CID byte 13 */
    tempbyte = (uint8_t)((sd_cid[3] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_cid.psn |= (uint32_t)tempbyte;

    /* CID byte 14 */
    tempbyte = (uint8_t)((sd_cid[3] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_cid.mdt = (uint16_t)tempbyte;

    /* CID byte 15 */
    tempbyte = (uint8_t)(sd_cid[3] & MMC_MASK_0_7BITS);
    pcardinfo->card_cid.cid_crc = (tempbyte & 0xFE) >> 1;

    /* CSD byte 0 */
    tempbyte = (uint8_t)((sd_csd[0] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_csd.csd_struct = (tempbyte & 0xC0) >> 6;
    pcardinfo->card_csd.spec_vers = (tempbyte & 0x3C) >> 2;

    /* CSD byte 1 */
    tempbyte = (uint8_t)((sd_csd[0] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_csd.taac = tempbyte;

    /* CSD byte 2 */
    tempbyte = (uint8_t)((sd_csd[0] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_csd.nsac = tempbyte;

    /* CSD byte 3 */
    tempbyte = (uint8_t)(sd_csd[0] & MMC_MASK_0_7BITS);
    pcardinfo->card_csd.tran_speed = tempbyte;

    /* CSD byte 4 */
    tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_csd.ccc = (uint16_t)((uint16_t)tempbyte << 4);

    /* CSD byte 5 */
    tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_csd.ccc |= (uint16_t)((uint16_t)(tempbyte & 0xF0) >> 4);
    pcardinfo->card_csd.read_bl_len = tempbyte & 0x0F;

    /* CSD byte 6 */
    tempbyte = (uint8_t)((sd_csd[1] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_csd.read_bl_partial = (tempbyte & 0x80) >> 7;
    pcardinfo->card_csd.write_blk_misalign = (tempbyte & 0x40) >> 6;
    pcardinfo->card_csd.read_blk_misalign = (tempbyte & 0x20) >> 5;
    pcardinfo->card_csd.dsp_imp = (tempbyte & 0x10) >> 4;
    /* card is SDSC card, CSD version 1.0 */
    pcardinfo->card_csd.c_size = (uint32_t)((uint32_t)(tempbyte & 0x03) << 10);

    /* CSD byte 7 */
    tempbyte = (uint8_t)(sd_csd[1] & MMC_MASK_0_7BITS);
    pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)tempbyte << 2);

    /* CSD byte 8 */
    tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_csd.c_size |= (uint32_t)((uint32_t)(tempbyte & 0xC0) >> 6);
    pcardinfo->card_csd.vdd_r_curr_min = (tempbyte & 0x38) >> 3;
    pcardinfo->card_csd.vdd_r_curr_max = tempbyte & 0x07;

    /* CSD byte 9 */
    tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_csd.vdd_w_curr_min = (tempbyte & 0xE0) >> 5;
    pcardinfo->card_csd.vdd_w_curr_max = (tempbyte & 0x1C) >> 2;
    pcardinfo->card_csd.c_size_mult = (tempbyte & 0x03) << 1;

    /* CSD byte 10 */
    tempbyte = (uint8_t)((sd_csd[2] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_csd.c_size_mult |= (tempbyte & 0x80) >> 7;

    /* calculate the card block size and capacity */
    pcardinfo->card_blocksize = 1 << (pcardinfo->card_csd.read_bl_len);
    pcardinfo->card_capacity = pcardinfo->card_csd.c_size + 1;
    pcardinfo->card_capacity *= (1 << (pcardinfo->card_csd.c_size_mult + 2));
    pcardinfo->card_capacity *= pcardinfo->card_blocksize;

    pcardinfo->card_csd.erase_grp_size = (tempbyte & 0x7C) >> 2;
    pcardinfo->card_csd.erase_grp_mult = (tempbyte & 0x03) << 3;

    /* CSD byte 11 */
    tempbyte = (uint8_t)(sd_csd[2] & MMC_MASK_0_7BITS);

    pcardinfo->card_csd.erase_grp_mult |= (tempbyte & 0xE0) >> 5;
    pcardinfo->card_csd.wp_grp_size = (tempbyte & 0x1F);

    /* CSD byte 12 */
    tempbyte = (uint8_t)((sd_csd[3] & MMC_MASK_24_31BITS) >> 24);
    pcardinfo->card_csd.wp_grp_enable = (tempbyte & 0x80) >> 7;
    pcardinfo->card_csd.r2w_factor = (tempbyte & 0x1C) >> 2;
    pcardinfo->card_csd.write_bl_len = (tempbyte & 0x03) << 2;

    /* CSD byte 13 */
    tempbyte = (uint8_t)((sd_csd[3] & MMC_MASK_16_23BITS) >> 16);
    pcardinfo->card_csd.write_bl_len |= (tempbyte & 0xC0) >> 6;
    pcardinfo->card_csd.write_bl_partial = (tempbyte & 0x20) >> 5;
    pcardinfo->card_csd.content_prot_app = (tempbyte & 0x01);

    /* CSD byte 14 */
    tempbyte = (uint8_t)((sd_csd[3] & MMC_MASK_8_15BITS) >> 8);
    pcardinfo->card_csd.file_format_grp = (tempbyte & 0x80) >> 7;
    pcardinfo->card_csd.copy_flag = (tempbyte & 0x40) >> 6;
    pcardinfo->card_csd.perm_write_protect = (tempbyte & 0x20) >> 5;
    pcardinfo->card_csd.tmp_write_protect = (tempbyte & 0x10) >> 4;
    pcardinfo->card_csd.file_format = (tempbyte & 0x0C) >> 2;
    pcardinfo->card_csd.ecc_code = (tempbyte & 0x03);

    /* CSD byte 15 */
    tempbyte = (uint8_t)(sd_csd[3] & MMC_MASK_0_7BITS);
    pcardinfo->card_csd.csd_crc = (tempbyte & 0xFE) >> 1;

    return status;
}

/*!
    \brief      check if the command sent error occurs
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum cmdsent_error_check(void)
{
    sd_error_enum status = MMC_OK;
    __IO uint32_t timeout = 100000;
    /* check command sent flag */
    while((RESET == sdio_flag_get(SDIO_FLAG_CMDSEND)) && (timeout > 0)) {
        --timeout;
    }
    /* command response is timeout */
    if(0 == timeout) {
        status = MMC_CMD_RESP_TIMEOUT;
        return status;
    }
    /* if the command is sent, clear the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    return status;
}

/*!
    \brief      check if error type for R1 response
    \param[in]  resp: content of response
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r1_error_type_check(uint32_t resp)
{
    sd_error_enum status = MMC_ERROR;
    /* check which error occurs */
    if(resp & MMC_R1_OUT_OF_RANGE) {
        status = MMC_OUT_OF_RANGE;
    } else if(resp & MMC_R1_ADDRESS_ERROR) {
        status = MMC_ADDRESS_ERROR;
    } else if(resp & MMC_R1_BLOCK_LEN_ERROR) {
        status = MMC_BLOCK_LEN_ERROR;
    } else if(resp & MMC_R1_ERASE_SEQ_ERROR) {
        status = MMC_ERASE_SEQ_ERROR;
    } else if(resp & MMC_R1_ERASE_PARAM) {
        status = MMC_ERASE_PARAM;
    } else if(resp & MMC_R1_WP_VIOLATION) {
        status = MMC_WP_VIOLATION;
    } else if(resp & MMC_R1_LOCK_UNLOCK_FAILED) {
        status = MMC_LOCK_UNLOCK_FAILED;
    } else if(resp & MMC_R1_COM_CRC_ERROR) {
        status = MMC_COM_CRC_ERROR;
    } else if(resp & MMC_R1_ILLEGAL_COMMAND) {
        status = MMC_ILLEGAL_COMMAND;
    } else if(resp & MMC_R1_CARD_ECC_FAILED) {
        status = MMC_CARD_ECC_FAILED;
    } else if(resp & MMC_R1_CC_ERROR) {
        status = MMC_CC_ERROR;
    } else if(resp & MMC_R1_GENERAL_UNKNOWN_ERROR) {
        status = MMC_GENERAL_UNKNOWN_ERROR;
    } else if(resp & MMC_R1_CSD_OVERWRITE) {
        status = MMC_CSD_OVERWRITE;
    } else if(resp & MMC_R1_WP_ERASE_SKIP) {
        status = MMC_WP_ERASE_SKIP;
    } else if(resp & MMC_R1_CARD_ECC_DISABLED) {
        status = MMC_CARD_ECC_DISABLED;
    } else if(resp & MMC_R1_ERASE_RESET) {
        status = MMC_ERASE_RESET;
    } else if(resp & MMC_R1_AKE_SEQ_ERROR) {
        status = MMC_AKE_SEQ_ERROR;
    }
    return status;
}

/*!
    \brief      check if error occurs for R1 response
    \param[in]  cmdindex: the index of command
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r1_error_check(uint8_t cmdindex)
{
    sd_error_enum status = MMC_OK;
    uint32_t reg_status = 0, resp_r1 = 0;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT;
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT;
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = MMC_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = MMC_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
        return status;
    }

    /* check whether the last response command index is the desired one */
    if(sdio_command_index_get() != cmdindex) {
        status = MMC_ILLEGAL_COMMAND;
        return status;
    }
    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    /* get the SDIO response register 0 for checking */
    resp_r1 = sdio_response_get(SDIO_RESPONSE0);
    if(MMC_ALLZERO == (resp_r1 & MMC_R1_ERROR_BITS)) {
        /* no error occurs, return MMC_OK */
        status = MMC_OK;
        return status;
    }

    /* if some error occurs, return the error type */
    status = r1_error_type_check(resp_r1);
    return status;
}

/*!
    \brief      check if error occurs for R2 response
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r2_error_check(void)
{
    sd_error_enum status = MMC_OK;
    uint32_t reg_status = 0;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT;
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT;
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = MMC_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = MMC_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
        return status;
    }
    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    return status;
}

/*!
    \brief      check if error occurs for R3 response
    \param[in]  none
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum r3_error_check(void)
{
    sd_error_enum status = MMC_OK;
    uint32_t reg_status = 0;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT;
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT;
    }
    if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = MMC_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
        return status;
    }
    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    return status;
}

/*!
    \brief      check if error occurs for R6 response
    \param[in]  cmdindex: the index of command
    \param[out] prca: a pointer that store the RCA of card
    \retval     sd_error_enum
*/
static sd_error_enum r6_error_check(uint8_t cmdindex, uint16_t *prca)
{
    sd_error_enum status = MMC_OK;
    uint32_t reg_status = 0, response = 0;

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT;
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT;
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = MMC_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = MMC_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
        return status;
    }

    /* check whether the last response command index is the desired one */
    if(sdio_command_index_get() != cmdindex) {
        status = MMC_ILLEGAL_COMMAND;
        return status;
    }
    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    /* get the SDIO response register 0 for checking */
    response = sdio_response_get(SDIO_RESPONSE0);

    if(MMC_ALLZERO == (response & (MMC_R6_COM_CRC_ERROR | MMC_R6_ILLEGAL_COMMAND | MMC_R6_GENERAL_UNKNOWN_ERROR))) {
        *prca = (uint16_t)(response >> 16);
        return status;
    }
    /* if some error occurs, return the error type */
    if(response & MMC_R6_COM_CRC_ERROR) {
        status = MMC_COM_CRC_ERROR;
    } else if(response & MMC_R6_ILLEGAL_COMMAND) {
        status = MMC_ILLEGAL_COMMAND;
    } else if(response & MMC_R6_GENERAL_UNKNOWN_ERROR) {
        status = MMC_GENERAL_UNKNOWN_ERROR;
    }
    return status;
}

/*!
    \brief      get the state which the card is in
    \param[in]  none
    \param[out] pcardstate: a pointer that store the card state
      \arg        MMC_CARDSTATE_IDLE: card is in idle state
      \arg        MMC_CARDSTATE_READY: card is in ready state
      \arg        MMC_CARDSTATE_IDENTIFICAT: card is in identificat state
      \arg        MMC_CARDSTATE_STANDBY: card is in standby state
      \arg        MMC_CARDSTATE_TRANSFER: card is in transfer state
      \arg        MMC_CARDSTATE_DATA: card is in data state
      \arg        MMC_CARDSTATE_RECEIVING: card is in receiving state
      \arg        MMC_CARDSTATE_PROGRAMMING: card is in programming state
      \arg        MMC_CARDSTATE_DISCONNECT: card is in disconnect state
      \arg        MMC_CARDSTATE_LOCKED: card is in locked state
    \retval     sd_error_enum
*/
static sd_error_enum sd_card_state_get(uint8_t *pcardstate)
{
    sd_error_enum status = MMC_OK;
    __IO uint32_t reg_status = 0, response = 0;

    /* send CMD13(SEND_STATUS), addressed card sends its status register */
    sdio_command_response_config(MMC_CMD_SEND_STATUS, (uint32_t)sd_rca << MMC_RCA_SHIFT, SDIO_RESPONSETYPE_SHORT);
    sdio_wait_type_set(SDIO_WAITTYPE_NO);
    sdio_csm_enable();

    /* store the content of SDIO_STAT */
    reg_status = SDIO_STAT;
    while(!(reg_status & (SDIO_FLAG_CCRCERR | SDIO_FLAG_CMDTMOUT | SDIO_FLAG_CMDRECV))) {
        reg_status = SDIO_STAT;
    }
    /* check whether an error or timeout occurs or command response received */
    if(reg_status & SDIO_FLAG_CCRCERR) {
        status = MMC_CMD_CRC_ERROR;
        sdio_flag_clear(SDIO_FLAG_CCRCERR);
        return status;
    } else if(reg_status & SDIO_FLAG_CMDTMOUT) {
        status = MMC_CMD_RESP_TIMEOUT;
        sdio_flag_clear(SDIO_FLAG_CMDTMOUT);
        return status;
    }

    /* command response received, store the response command index */
    reg_status = (uint32_t)sdio_command_index_get();
    if(reg_status != (uint32_t)MMC_CMD_SEND_STATUS) {
        status = MMC_ILLEGAL_COMMAND;
        return status;
    }
    /* clear all the SDIO_INTC flags */
    sdio_flag_clear(SDIO_MASK_INTC_FLAGS);
    /* get the SDIO response register 0 for checking */
    response = sdio_response_get(SDIO_RESPONSE0);
    *pcardstate = (uint8_t)((response >> 9) & 0x0000000F);

    if(MMC_ALLZERO == (response & MMC_R1_ERROR_BITS)) {
        /* no error occurs, return MMC_OK */
        status = MMC_OK;
        return status;
    }

    /* if some error occurs, return the error type */
    status = r1_error_type_check(response);
    return status;
}

/*!
    \brief      configure the bus width mode
    \param[in]  buswidth: the bus width
      \arg        MMC_BUS_WIDTH_1BIT: 1-bit bus width
      \arg        MMC_BUS_WIDTH_4BIT: 4-bit bus width
    \param[out] none
    \retval     sd_error_enum
*/
static sd_error_enum sd_bus_width_config(uint32_t buswidth)
{
    sd_error_enum status = MMC_OK;
    /* check whether the card is locked */
    if(sdio_response_get(SDIO_RESPONSE0) & MMC_CARDSTATE_LOCKED) {
        status = MMC_LOCK_UNLOCK_FAILED;
        return status;
    }
    return status;
}

/*!
    \brief      get the data block size
    \param[in]  bytesnumber: the number of bytes
    \param[out] none
    \retval     data block size
      \arg        SDIO_DATABLOCKSIZE_1BYTE: block size = 1 byte
      \arg        SDIO_DATABLOCKSIZE_2BYTES: block size = 2 bytes
      \arg        SDIO_DATABLOCKSIZE_4BYTES: block size = 4 bytes
      \arg        SDIO_DATABLOCKSIZE_8BYTES: block size = 8 bytes
      \arg        SDIO_DATABLOCKSIZE_16BYTES: block size = 16 bytes
      \arg        SDIO_DATABLOCKSIZE_32BYTES: block size = 32 bytes
      \arg        SDIO_DATABLOCKSIZE_64BYTES: block size = 64 bytes
      \arg        SDIO_DATABLOCKSIZE_128BYTES: block size = 128 bytes
      \arg        SDIO_DATABLOCKSIZE_256BYTES: block size = 256 bytes
      \arg        SDIO_DATABLOCKSIZE_512BYTES: block size = 512 bytes
      \arg        SDIO_DATABLOCKSIZE_1024BYTES: block size = 1024 bytes
      \arg        SDIO_DATABLOCKSIZE_2048BYTES: block size = 2048 bytes
      \arg        SDIO_DATABLOCKSIZE_4096BYTES: block size = 4096 bytes
      \arg        SDIO_DATABLOCKSIZE_8192BYTES: block size = 8192 bytes
      \arg        SDIO_DATABLOCKSIZE_16384BYTES: block size = 16384 bytes
*/
static uint32_t sd_datablocksize_get(uint16_t bytesnumber)
{
    uint8_t exp_val = 0;
    /* calculate the exponent of 2 */
    while(1 != bytesnumber) {
        bytesnumber >>= 1;
        ++exp_val;
    }
    return DATACTL_BLKSZ(exp_val);
}

/*!
    \brief      configure the GPIO of SDIO interface
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void gpio_config(void)
{
    /* configure the SDIO_D0(PC8), SDIO_D1(PB0), SDIO_D2(PB1), SDIO_D3(PC11), SDIO_D4(PB8),
       SDIO_D5(PB9), SDIO_D6(PC6), SDIO_D7(PC7), SDIO_CLK(PB2) and SDIO_CMD(PA6) */
    gpio_af_set(GPIOB, GPIO_AF_12, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_8 | GPIO_PIN_9);
    gpio_af_set(GPIOC, GPIO_AF_12, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11);  //GPIO_PIN_9 | GPIO_PIN_10 |
    gpio_af_set(GPIOA, GPIO_AF_12, GPIO_PIN_6);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9);

    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11); //GPIO_PIN_9 | GPIO_PIN_10 |
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_11); //GPIO_PIN_9 | GPIO_PIN_10 |


    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_2);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_6);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PIN_6);
}

/*!
    \brief      configure the RCU of SDIO and DMA
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void rcu_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOC);

    rcu_periph_clock_enable(RCU_SDIO);
    rcu_periph_clock_enable(RCU_DMA1);
}


/*!
    \brief      configure the DMA1 channel 3 for transferring data
    \param[in]  srcbuf: a pointer point to a buffer which will be transferred
    \param[in]  bufsize: the size of buffer(not used in flow controller is peripheral)
    \param[out] none
    \retval     none
*/

static void dma_transfer_config(uint32_t *srcbuf, uint32_t bufsize)
{
    dma_multi_data_parameter_struct dma_struct;
    /* clear all the interrupt flags */
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_FEE);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_SDE);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_TAE);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_HTF);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_FTF);
    dma_channel_disable(DMA1, DMA_CH3);
    dma_deinit(DMA1, DMA_CH3);

    /* configure the DMA1 channel 3 */
    dma_struct.periph_addr = (uint32_t)SDIO_FIFO_ADDR;
    dma_struct.memory0_addr = (uint32_t)srcbuf;
    dma_struct.direction = DMA_MEMORY_TO_PERIPH;
    dma_struct.number = 0;
    dma_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_struct.periph_width = DMA_PERIPH_WIDTH_32BIT;
    dma_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_struct.periph_burst_width = DMA_PERIPH_BURST_4_BEAT;
    dma_struct.memory_burst_width = DMA_MEMORY_BURST_4_BEAT;
    dma_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
    dma_struct.critical_value = DMA_FIFO_4_WORD;
    dma_multi_data_mode_init(DMA1, DMA_CH3, &dma_struct);
    
    dma_flow_controller_config(DMA1, DMA_CH3, DMA_FLOW_CONTROLLER_PERI);
    dma_channel_subperipheral_select(DMA1, DMA_CH3, DMA_SUBPERI4);
    dma_channel_enable(DMA1, DMA_CH3);
}

/*!
\brief      configure the DMA1 channel 3 for receiving data
\param[in]  dstbuf: a pointer point to a buffer which will receive data
\param[in]  bufsize: the size of buffer(not used in flow controller is peripheral)
\param[out] none
\retval     none
*/
static void dma_receive_config(uint32_t *dstbuf, uint32_t bufsize)
{
    dma_multi_data_parameter_struct dma_struct;
    /* clear all the interrupt flags */
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_FEE);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_SDE);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_TAE);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_HTF);
    dma_flag_clear(DMA1, DMA_CH3, DMA_FLAG_FTF);
    dma_channel_disable(DMA1, DMA_CH3);
    dma_deinit(DMA1, DMA_CH3);

    /* configure the DMA1 channel 3 */
    dma_struct.periph_addr = (uint32_t)SDIO_FIFO_ADDR;
    dma_struct.memory0_addr = (uint32_t)dstbuf;
    dma_struct.direction = DMA_PERIPH_TO_MEMORY;
    dma_struct.number = 0;
    dma_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_struct.periph_width = DMA_PERIPH_WIDTH_32BIT;
    dma_struct.memory_width = DMA_MEMORY_WIDTH_32BIT;
    dma_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    dma_struct.periph_burst_width = DMA_PERIPH_BURST_4_BEAT;
    dma_struct.memory_burst_width = DMA_MEMORY_BURST_4_BEAT;
    dma_struct.critical_value = DMA_FIFO_4_WORD;
    dma_struct.circular_mode = DMA_CIRCULAR_MODE_DISABLE;
    dma_multi_data_mode_init(DMA1, DMA_CH3, &dma_struct);
    
    dma_flow_controller_config(DMA1, DMA_CH3, DMA_FLOW_CONTROLLER_PERI);
    dma_channel_subperipheral_select(DMA1, DMA_CH3, DMA_SUBPERI4);
    dma_channel_enable(DMA1, DMA_CH3);
}

