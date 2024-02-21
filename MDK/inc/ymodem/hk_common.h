/*******************************************************************************
 ** Copyright (C) 2021, SICHUAN HUAKUN ZHENYU INTELLIGENT TECHNOLOGY CO.,LTD.
 ** All rights reserved.
 **
 ** \file   hk_common.h
 **
 ** \brief  include all common tools. for example,hk_print_msg, data processing tools etc...
 **
 ** \author
 **
 ******************************************************************************/

/*******************************************************************************
 * Include files
 ******************************************************************************/
#ifndef __HK_COMMON_H__
#define __HK_COMMON_H__

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;

#define IS_CAP_LETTER(c) (((c) >= 'A') && ((c) <= 'F'))
#define IS_LC_LETTER(c) (((c) >= 'a') && ((c) <= 'f'))
#define IS_09(c) (((c) >= '0') && ((c) <= '9'))
#define ISVALIDHEX(c) (IS_CAP_LETTER(c) || IS_LC_LETTER(c) || IS_09(c))
#define ISVALIDDEC(c) IS_09(c)
#define CONVERTDEC(c) (c - '0')

#define CONVERTHEX_ALPHA(c) (IS_CAP_LETTER(c) ? ((c) - 'A' + 10) : ((c) - 'a' + 10))
#define CONVERTHEX(c) (IS_09(c) ? ((c) - '0') : CONVERTHEX_ALPHA(c))
#define MEM_ZERO_STRUCT(x)                     \
    do                                         \
    {                                          \
        memset((void *)&(x), 0l, (sizeof(x))); \
    } while (0)



typedef enum
{
    HK_OK = 0u, ///< No error
    HK_EEROR_INT = -0x4000,
} hk_result_t;

uint16_t modbus_crc16(uint8_t *pdata, uint16_t len);
uint16_t cal_crc16(const uint8_t *p_data, uint32_t size);
uint8_t calc_checksum(const uint8_t *p_data, uint32_t size);
void int2str(uint8_t *p_str, uint32_t intnum);
uint32_t str2int(uint8_t *p_inputstr, uint32_t *p_intnum);


 enum
 {
     HK_OFF = 0,
     HK_ERROR,
     HK_WARNING,
     HK_INFO,
     HK_DEBUG,
     HK_ALWAYS,
     HK_NOTHING
 };
 extern int dbg_level;
 #define hk_print_msg(level, ...)                        \
     do                                                  \
     {                                                   \
         if (level > dbg_level)                          \
             break;                                      \
         if (level == HK_INFO)                           \
             printf(" [INFO]: ");                        \
         else if (level == HK_DEBUG)                     \
             printf(" [DBG]: ");                         \
         else if (level == HK_WARNING)                   \
             printf(" [WARN]: ");                        \
         else if (level == HK_ERROR)                     \
             printf(" [ERR]: ");                         \
         printf("[%s]->[%d]: ", __FUNCTION__, __LINE__); \
         printf(__VA_ARGS__);                            \
         printf("\r\n");                                 \
     } while (0)

#endif

