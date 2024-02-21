#ifndef __HK_YMODEM_H__
#define __HK_YMODEM_H__

#include <stdio.h>
#include <string.h>
// #include "hk_ymodem.h"

/* 流程及协议相关宏和结构体 */
#define SOH (0x01)   /* start of 128-byte data packet */
#define STX (0x02)   /* start of 1024-byte data packet */
#define EOT (0x04)   /* end of transmission */
#define ACK (0x06)   /* acknowledge */
#define NAK (0x15)   /* negative acknowledge */
#define CA (0x18)    /* two of these in succession aborts transfer */
#define CRC16 (0x43) /* 'C' == 0x43, request 16-bit CRC */
#define NEGATIVE_BYTE ((uint8_t)0xFF)

#define ABORT1 (0x41) /* 'A' == 0x41, abort by user */
#define ABORT2 (0x61) /* 'a' == 0x61, abort by user */

// #define PKT_HEADER_SIZE ((uint32_t)3)
// #define PKT_DATA_INDEX ((uint32_t)4)
// #define PKT_START_INDEX ((uint32_t)1)
// #define PKT_NUMBER_INDEX ((uint32_t)2)
// #define PKT_CNUMBER_INDEX ((uint32_t)3)
// #define PKT_TRAILER_SIZE ((uint32_t)2)
// #define PKT_OVERHEAD_SIZE (PKT_HEADER_SIZE + PKT_TRAILER_SIZE) // zy+-1

#define PACKET_SEQNO_INDEX (1)
#define PACKET_SEQNO_COMP_INDEX (2)

#define PACKET_HEADER (3)
#define PACKET_TRAILER (2)
#define PACKET_OVERHEAD (PACKET_HEADER + PACKET_TRAILER)

#define FILE_NAME_LENGTH ((uint32_t)64)
#define FILE_SIZE_LENGTH ((uint32_t)16)

#define PKT_DATA_1K 1024
#define PKT_DATA_128 128
#define PKT_TRY_TIMES 5

#define PKT_MODE_2_DATA_LEN(_MODE) ((STX == _MODE) ? PKT_DATA_1K : PKT_DATA_128)
#define ymodem_print(...)
    //                          \
    //                          \
    // do                       \
    // {                        \
    //     printf(__VA_ARGS__); \
    //     printf("\r\n");      \
    // } while (0)

typedef struct
{
    char mode;
    char id;
    char id2;
    char data_buf[PKT_DATA_1K];
    int data_len;
    int crc16;
} YMODEM_PKT;

/* 返回值 */
enum
{
    YMODEM_OK = 0,
 
    YMODEM_E_PARA = -1,
    YMODEM_E_SIZE = -2,
    YMODEM_E_FILL = -3,
    YMODEM_E_RETRY = -4,
    YMODEM_E_BUSY = -5,
    YMODEM_E_RECV = -6,
    YMODEM_E_ABORT = -7,
};

//#pragma region ymodem_send

/* 输入输出接口原型 */
typedef void (*YMODEM_PUTC_FUN)(char);
typedef int (*YMODEM_GETC_FUN)();
typedef int (*YMODEM_CRC16_FUN)(char *buf, int len);
typedef void (*YMODEM_DELAY_FUN)(int);

/* 初始化输入输出接口 */
void ymodem_io_init(YMODEM_PUTC_FUN putc, YMODEM_GETC_FUN getc, YMODEM_CRC16_FUN crc16, YMODEM_DELAY_FUN delay);

/* 传输文件 */
int ymodem_file_transfer(char *file_name, char *file_ptr, int file_len);
int xmodem_file_transfer(char *file_ptr, int file_len);

void ymodem_update_task(void);

int ymodem_file_recvive(char *file_name, char *file_data, int *file_len);
void ymodem_update_thread(void);


//#pragma endregion

#endif
