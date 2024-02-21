/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   hk_common.c
 **
 ** \brief  include all common tools. for example,hk_print_msg, data processing tools etc...
 **
 ** \author
 **
 ******************************************************************************/

#include "hk_common.h"

/*******************************************************************************
 * Include files
 ******************************************************************************/

/**
 * @brief  Update CRC16 for input byte
 * @param  crc_in input value
 * @param  input byte
 * @retval None
 */
uint16_t modbus_crc16(uint8_t *pdata, uint16_t len)
{
    uint16_t i, j, crc;
    crc = 0xFFFF;
    for (j = 0; j < len; j++)
    {
        crc = crc ^ pdata[j];
        for (i = 0; i < 8; i++)
        {
            if ((crc & 0x0001) > 0)
            {
                crc = crc >> 1;
                crc = crc ^ 0xa001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }
    return crc;
}

/**
 * @brief  Update CRC16 for input byte
 * @param  crc_in input value
 * @param  input byte
 * @retval None
 */
static uint16_t update_crc16(uint16_t crc_in, uint8_t byte)
{
    uint32_t crc = crc_in;
    uint32_t in = byte | 0x100;

    do
    {
        crc <<= 1;
        in <<= 1;
        if (in & 0x100)
            ++crc;
        if (crc & 0x10000)
            crc ^= 0x1021;
    }

    while (!(in & 0x10000));

    return crc & 0xffffu;
}

/**
 * @brief  Cal CRC16 for YModem Packet
 * @param  data
 * @param  length
 * @retval None
 */
uint16_t cal_crc16(const uint8_t *p_data, uint32_t size)
{
    uint32_t crc = 0;
    const uint8_t *dataEnd = p_data + size;

    while (p_data < dataEnd)
        crc = update_crc16(crc, *p_data++);

    crc = update_crc16(crc, 0);
    crc = update_crc16(crc, 0);

    return crc & 0xffffu;
}

/**
 * @brief  Calculate Check sum for YModem Packet
 * @param  p_data Pointer to input data
 * @param  size length of input data
 * @retval uint8_t checksum value
 */
uint8_t calc_checksum(const uint8_t *p_data, uint32_t size)
{
    uint32_t sum = 0;
    const uint8_t *p_data_end = p_data + size;

    while (p_data < p_data_end)
    {
        sum += *p_data++;
    }

    return (sum & 0xffu);
}




/**
 * @brief  Convert an Integer to a string
 * @param  p_str: The string output pointer
 * @param  intnum: The integer to be converted
 * @retval None
 */
void int2str(uint8_t *p_str, uint32_t intnum)
{
    uint32_t i, divider = 1000000000, pos = 0, status = 0;

    for (i = 0; i < 10; i++)
    {
        p_str[pos++] = (intnum / divider) + 48;

        intnum = intnum % divider;
        divider /= 10;
        if ((p_str[pos - 1] == '0') & (status == 0))
        {
            pos = 0;
        }
        else
        {
            status++;
        }
    }
}

/**
 * @brief  Convert a string to an integer
 * @param  p_inputstr: The string to be converted
 * @param  p_intnum: The integer value
 * @retval 1: Correct
 *         0: Error
 */
uint32_t str2int(uint8_t *p_inputstr, uint32_t *p_intnum)
{
    uint32_t i = 0, res = 0;
    uint32_t val = 0;

    if ((p_inputstr[0] == '0') && ((p_inputstr[1] == 'x') || (p_inputstr[1] == 'X')))
    {
        i = 2;
        while ((i < 11) && (p_inputstr[i] != '\0'))
        {
            if (ISVALIDHEX(p_inputstr[i]))
            {
                val = (val << 4) + CONVERTHEX(p_inputstr[i]);
            }
            else
            {
                /* Return 0, Invalid input */
                res = 0;
                break;
            }
            i++;
        }

        /* valid result */
        if (p_inputstr[i] == '\0')
        {
            *p_intnum = val;
            res = 1;
        }
    }
    else /* max 10-digit decimal input */
    {
        while ((i < 11) && (res != 1))
        {
            if (p_inputstr[i] == '\0')
            {
                *p_intnum = val;
                /* return 1 */
                res = 1;
            }
            else if (((p_inputstr[i] == 'k') || (p_inputstr[i] == 'K')) && (i > 0))
            {
                val = val << 10;
                *p_intnum = val;
                res = 1;
            }
            else if (((p_inputstr[i] == 'm') || (p_inputstr[i] == 'M')) && (i > 0))
            {
                val = val << 20;
                *p_intnum = val;
                res = 1;
            }
            else if (ISVALIDDEC(p_inputstr[i]))
            {
                val = val * 10 + CONVERTDEC(p_inputstr[i]);
            }
            else
            {
                /* return 0, Invalid input */
                res = 0;
                break;
            }
            i++;
        }
    }

    return res;
}
