#include "hk_ymodem.h"
#include "gd32f4xx.h"
#include "gd32f4xx_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "hk_common.h"
#include "fatfs/fatfs_user.h"
#include "upgrade/bootload.h"
#include "wdg/hk_wdg.h"
#include "pin.h"
/*
 * 以下是自定义io，需根据自己平台设置
 */

/* 初始化输入输出接口 */
static YMODEM_PUTC_FUN ymodem_putc_fun = NULL;
static YMODEM_GETC_FUN ymodem_getc_fun = NULL;
static YMODEM_CRC16_FUN ymodem_crc16_fun = NULL;
static YMODEM_DELAY_FUN ymodem_delay_fun = NULL;

static void printStr(uint8_t *cChar)
{
    while (*cChar)
    {
        usart_data_transmit(UART4, *cChar);
        while (RESET == usart_flag_get(UART4, USART_FLAG_TBE))
            ;
        *cChar++;
    }
}

int ymodem_debug = 0;
void ymodem_io_init(YMODEM_PUTC_FUN putc, YMODEM_GETC_FUN getc, YMODEM_CRC16_FUN crc16, YMODEM_DELAY_FUN delay)
{
    ymodem_putc_fun = putc;
    ymodem_getc_fun = getc;
    ymodem_crc16_fun = crc16;
    ymodem_delay_fun = delay;
}

/* 输出字符 */
static void ymodem_putc(char c)
{
    if (ymodem_putc_fun)
        ymodem_putc_fun(c);
    else
        ymodem_print("\r\n Need: ymodem_io_init()");

    if ((ymodem_debug & 0x1) == 0x1)
        ymodem_print("t: %02x ", c);
}

/* 输入字符 */
static int ymodem_getc()
{
    int c = -1;

    if (ymodem_getc_fun)
        c = ymodem_getc_fun();
    else
        ymodem_print("\r\n Need: ymodem_io_init()");

    if ((ymodem_debug & 0x2) == 0x2)
        ymodem_print("\r\nr: %02x \r\n", c);

    return c;
}

/* crc计算 */
static int ymodem_crc16(char *buf, int len)
{
    int rc = 0;

    if (ymodem_crc16_fun)
        rc = ymodem_crc16_fun(buf, len);
    else
        ymodem_print("\r\n Need: ymodem_io_init()");

    return rc;
}

/* 延时 */
static void ymodem_delay(int ticks)
{
    if (ymodem_delay_fun)
        ymodem_delay_fun(ticks);
    else
        ymodem_print("\r\n Need: ymodem_io_init()");
}

/* 报文填充 */
static int ymodem_pkt_fill(int pkt_mode, int pkt_id, char *a_data_ptr, int a_data_len, YMODEM_PKT *pkt)
{
    int pkt_data_len = PKT_MODE_2_DATA_LEN(pkt_mode);

    if ((pkt_id < 0) || (NULL == a_data_ptr) || (a_data_len <= 0) || (NULL == pkt))
        return YMODEM_E_PARA;

    // 头: 发送模式、报文id、报文id补码
    pkt->mode = pkt_mode;
    if (EOT != pkt_mode)
    {
        pkt->id = pkt_id;
        pkt->id2 = ~pkt_id;
        // 数据
        memset(pkt->data_buf, 0, sizeof(pkt->data_buf));
        memcpy(pkt->data_buf, a_data_ptr, a_data_len);
        pkt->data_len = pkt_data_len;
        // crc
        pkt->crc16 = ymodem_crc16(pkt->data_buf, pkt->data_len);
    }

    return YMODEM_OK;
}

/* 报文发送 */
static void ymodem_pkt_tx(YMODEM_PKT pkt)
{
    int i;

    ymodem_putc(pkt.mode);

    // 非eot
    if (EOT != pkt.mode)
    {
        // //printf("\r\n pkt_mode %d, pkt_id %d, pkt_data_len %d, pkt.crc16 %x\r\n", pkt.mode, pkt.id, pkt.data_len, pkt.crc16);
        ymodem_putc(pkt.id);
        ymodem_putc(pkt.id2);

        for (i = 0; i < pkt.data_len; i++)
        {
            ymodem_putc(pkt.data_buf[i]);
        }
        ymodem_putc(pkt.crc16 >> 8);
        ymodem_putc(pkt.crc16 & 0xFF);
    }

    // 发送等待
    // ymodem_delay(5);
}

/* 报文回应确认: 返回是否ok */
static int ymodem_pkt_ack(YMODEM_PKT pkt)
{
    int pkt_first = 0;
    int pkt_end = 0;
    char pkt_ack, ack_crc16;

    // //判断pkt标记
    if ((0 == pkt.id) && (0 != pkt.data_buf[0]))
        pkt_first = 1;
    else if (EOT == pkt.mode)
        pkt_end = 1;

    // 接收等待, 对端处理能力有限
    // if (pkt_first)
    //     ymodem_delay(20);
    // else
    //     ymodem_delay(5);

    // 接收
    pkt_ack = ymodem_getc();
    if (ACK != pkt_ack)
        return 0;
    // 开始和结束会返回crc
    if (pkt_first || pkt_end)
    {
        ack_crc16 = ymodem_getc();
    }

    return 1;
}

/* 报文传输 */
static int ymodem_pkt_transfer(int pkt_mode, int pkt_id, char *a_data_ptr, int a_data_len)
{
    YMODEM_PKT pkt;
    int i = 0;

    if ((pkt_id < 0) || (NULL == a_data_ptr) || (a_data_len <= 0))
        return YMODEM_E_PARA;

    // 填充
    if (0 != ymodem_pkt_fill(pkt_mode, pkt_id, a_data_ptr, a_data_len, &pkt))
        return YMODEM_E_FILL;

    // //传输, 其中ymodem的两个eot报文通过重传实现
    for (i = 0; i <= PKT_TRY_TIMES; i++)
    {
        // 发送
        ymodem_pkt_tx(pkt);

        // 应答确认
        if (ymodem_pkt_ack(pkt))
            break;
    }
    // 传输失败
    if (i > PKT_TRY_TIMES)
        return YMODEM_E_RETRY;

    // 重传次数
    return i;
}

/* 数据分片报文传输 */
static int ymodem_pkts_transfer(int pkt_mode, int pkt_id_base, char *data_ptr, int data_len)
{
    int rc, data_offset, remain_len;
    int pkt_data_len;
    int pkt_id;
    char *a_data_ptr;
    int a_data_len;

    if ((pkt_id_base < 0) || (NULL == data_ptr) || (data_len <= 0))
        return YMODEM_E_PARA;

    pkt_id = pkt_id_base;
    pkt_data_len = PKT_MODE_2_DATA_LEN(pkt_mode);
    for (data_offset = 0; data_offset < data_len;)
    {
        remain_len = data_len - data_offset;
#if 0
        //如果1k模式不够1k，更改报文数据长度为128
        if((STX == pkt_mode) && (remain_len < PKT_DATA_1K))
        {
            pkt_data_len = PKT_DATA_128;
        }
        ////printf("\r\n pkt_id %d, pkt_data_len %d, data_offset %d\r\n", pkt_id, pkt_data_len, data_offset);
#endif

        // 传输一个
        a_data_ptr = data_ptr + data_offset;
        a_data_len = (remain_len > pkt_data_len) ? pkt_data_len : remain_len;
        rc = ymodem_pkt_transfer(pkt_mode, pkt_id, a_data_ptr, a_data_len);
        if (rc < 0)
            return rc;

        // 下一个报文
        pkt_id++;
        data_offset += pkt_data_len;
    }

    return YMODEM_OK;
}

/* 传输控制报文 */
static int ymodem_ctrl_transfer(int pkt_mode, char *file_name, int file_len)
{
    char data_buf[PKT_DATA_128] = {0};

    if ((NULL != file_name) && (file_len > 0))
    {
        // 填充文件名和长度
        sprintf(data_buf, "%s %d", file_name, file_len);
        data_buf[strlen(file_name)] = 0;
    }

    return ymodem_pkt_transfer(pkt_mode, 0, data_buf, sizeof(data_buf));
}

/* 传输数据报文 */
static int ymodem_data_transfer(int pkt_mode, char *data_ptr, int data_len)
{
    return ymodem_pkts_transfer(pkt_mode, 1, data_ptr, data_len);
}

/* 接收数据报文*/
static int ymodem_pkt_receive(char *p_data, int *p_data_len)
{
    uint32_t crc;
    uint32_t packet_size = 0;
    int ret = -1;
    int rx_char = -1;
    *p_data_len = 0;
    rx_char = ymodem_getc();
    if (rx_char < 0)
        return YMODEM_E_RECV;
    switch (rx_char)
    {
    case SOH:
        packet_size = PKT_DATA_128;
        break;
    case STX:
        packet_size = PKT_DATA_1K;
        break;
    case EOT:
        return YMODEM_OK;

    case CA:
        rx_char = ymodem_getc();
        if (rx_char == CA)
        {
            *p_data_len = -1;
            return YMODEM_OK;
        }
        else
        {
            ymodem_print("[%s]===%d,%d\r\n", __func__, __LINE__, rx_char);
            ret = YMODEM_E_RECV;
        }
        break;
    case ABORT1:
    case ABORT2:
        ymodem_print("[%s]===%d,%d\r\n", __func__, __LINE__, rx_char);
        ret = YMODEM_E_BUSY;
        break;
    default:
        ymodem_print("[%s]===%d,%d\r\n", __func__, __LINE__, rx_char);
        ret = YMODEM_E_RECV;
        break;
    }

    p_data[0] = rx_char;
    for (uint16_t i = 1; i < (packet_size + PACKET_OVERHEAD); i++)
    {
        rx_char = ymodem_getc();
        if (rx_char >= 0)
        {
            p_data[i] = (uint8_t)rx_char;
        }
        else
        {
            packet_size = 0;
            *p_data_len = packet_size;
            ymodem_print("[%s]===%d,%d\r\n", __func__, __LINE__, rx_char);
            return YMODEM_E_RECV;
        }
    }

    if (p_data[PACKET_SEQNO_INDEX] != ((p_data[PACKET_SEQNO_COMP_INDEX]) ^ NEGATIVE_BYTE))
    {
        *p_data_len = 0;
        ymodem_print("[%s]===%d,%d\r\n", __func__, __LINE__, rx_char);
        return YMODEM_E_RECV;
    }
    // else
    // {
    //     /* Check packet CRC */
    //     crc = p_data[packet_size + PKT_DATA_INDEX] << 8;
    //     crc += p_data[packet_size + PKT_DATA_INDEX + 1];
    //     if (ymodem_crc16((char *)&p_data[PKT_DATA_INDEX], packet_size) != crc)
    //     {
    //         packet_size = 0;
    //         *p_data_len = packet_size;
    //         printf("[%s]===%d,%d\r\n", __func__, __LINE__, rx_char);
    //         ret = YMODEM_E_RECV;
    //     }
    // }
    *p_data_len = packet_size;
    return YMODEM_OK;
}

/*
    传输文件(ymodem)
    注意: 控制报文采用soh(128字节), 数据报文采用stx(1k字节)
 */
int ymodem_file_transfer(char *file_name, char *file_ptr, int file_len)
{
    int rc;

    if ((NULL == file_name) || (NULL == file_ptr) || (file_len <= 0))
        return YMODEM_E_PARA;

    // 传输文件名和长度
    rc = ymodem_ctrl_transfer(SOH, file_name, file_len);
    if (rc < 0)
        return rc;

    // 传输文件内容
    rc = ymodem_data_transfer(STX, file_ptr, file_len);
    if (rc < 0)
        return rc;

    // 传输结束
    rc = ymodem_ctrl_transfer(EOT, NULL, 0);
    if (rc < 0)
        return rc;

    // 传输结束确认
    rc = ymodem_ctrl_transfer(SOH, NULL, 0);
    if (rc < 0)
        return rc;

    return YMODEM_OK;
}

/*
 传输文件(xmodem)
 注意: 控制报文采用soh(128字节), 数据报文采用stx(1k字节)
       根据ymodem扩展的，未测试
 */
int xmodem_file_transfer(char *file_ptr, int file_len)
{
    int rc;

    if ((NULL == file_ptr) || (file_len <= 0))
        return YMODEM_E_PARA;

    // 传输文件内容
    rc = ymodem_data_transfer(STX, file_ptr, file_len);
    if (rc < 0)
        return rc;

    // 传输结束
    rc = ymodem_ctrl_transfer(EOT, NULL, 0);
    if (rc < 0)
        return rc;

    return YMODEM_OK;
}

/* ymodem接收文件*/
/* TODO: del */

int ymodem_file_recvive(char *file_name, char *file_data, int *file_len)
{
    uint8_t packet_data[PKT_DATA_1K + PACKET_OVERHEAD] = {0}, file_size[FILE_SIZE_LENGTH] = {0}, *file_ptr = NULL, *buf_ptr = NULL;
    int i, packet_length, session_done, file_done, packets_received, errors, session_begin, size = 0;
    static int write_offset = 0;
    *file_len = -1;
    static char updata_file_name[FILE_NAME_LENGTH] = {0};
    for (session_done = 0, errors = 0, session_begin = 0;;)
    {
        for (packets_received = 0, file_done = 0, buf_ptr = file_data;;)
        {
            IWDG_FEED(); // 接收时喂狗
            memset(packet_data, 0, PKT_DATA_1K + PACKET_OVERHEAD);
            memset(file_size, 0, FILE_SIZE_LENGTH);
            switch (ymodem_pkt_receive(packet_data, &packet_length))
            {
            case YMODEM_OK:
                errors = 0;
                switch (packet_length)
                {
                /* Abort by sender */
                case -1:
                    ymodem_putc(ACK);
                    return 0;
                /* End of transmission */
                case 0:
                    ymodem_putc(ACK);
                    file_done = 1;
                    break;
                /* Normal packet */
                default:
                    if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0xff))
                    {
                        ymodem_putc(NAK);
                    }
                    else
                    {
                        if (packets_received == 0)
                        {
                            /* Filename packet */
                            if (packet_data[PACKET_HEADER] != 0)
                            {
                                /* Filename packet has valid data */
                                for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < FILE_NAME_LENGTH);)
                                {
                                    file_name[i++] = *file_ptr++;
                                }
                                file_name[i++] = '\0';
                                for (i = 0, file_ptr++; (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH);)
                                {
                                    file_size[i++] = *file_ptr++;
                                }
                                file_size[i++] = '\0';
                                str2int(file_size, &size);
                                *file_len = size;
                                /* Test the size of the image to be sent */
                                /* Image size is greater than Flash size */

                                if (size > (0x6000000 - 1)) //todo 文件大小限制 60M
                                {
                                    /* End session */
                                    ymodem_putc(CA);
                                    ymodem_putc(CA);
                                    return -1;
                                }
                                // !!!!!!!get  --->> data size & 擦除写入区域  todo+zy

                                uint8_t try_times = 10;
                                FRESULT ret = FR_INT_ERR;
                                memset(updata_file_name, 0, FILE_NAME_LENGTH);
                                if (strstr(file_name, "mcu-update"))
                                {
                                    memcpy(updata_file_name, UPLOAD_BMC_FILE, strlen(UPLOAD_BMC_FILE));
                                }
                                else if (strstr(file_name, "bios-update"))
                                {
                                    memcpy(updata_file_name, UPLOAD_BIOS_FILE, strlen(UPLOAD_BIOS_FILE));
                                }
                                else
                                {
                                    memcpy(updata_file_name, OTHER_TEST_FILE, strlen(OTHER_TEST_FILE));
                                }

                                while ((ret != FR_OK) && (try_times > 1))
                                {
                                    ret = f_unlink(updata_file_name);
                                    vTaskDelay(500);
                                    try_times--;
                                }

                                if (ret == FR_OK || ret == FR_NO_FILE)
                                {
                                    ymodem_putc(ACK);
                                    ymodem_putc(CRC16);
                                }
                                else
                                {
                                    ymodem_putc(CA);
                                    ymodem_putc(CA);
                                    return YMODEM_E_BUSY;
                                }

                                // ymodem_putc(ACK);
                                // ymodem_putc(CRC16);
                            }
                            /* Filename packet is empty, end session */
                            else
                            {
                                ymodem_putc(ACK);
                                file_done = 1;
                                session_done = 1;
                                break;
                            }
                        }
                        /* Data packet */
                        else
                        {
                            // !!!!!!!!!!!!!!!!!!!!!!send pkt data，写入区域
                            // memcpy(buf_ptr, packet_data + PACKET_HEADER, packet_length);

                            FRESULT ret = FR_INT_ERR;
                            uint8_t try_times = 10;
                            while ((ret != FR_OK) && (try_times > 1))
                            {
                                ret = fatfs_write_data(updata_file_name, 1, write_offset, packet_data + PACKET_HEADER, packet_length);
                                try_times--;
                            }

                            if (ret == FR_OK)
                            {
                                write_offset += packet_length;
                                ymodem_putc(ACK);
                            }
                            else
                            {
                                ymodem_putc(CA);
                                ymodem_putc(CA);

                                return YMODEM_E_BUSY;
                            }
                        }

                        packets_received++;
                        session_begin = 1;
                    }
                    break;
                }
                break;

            case YMODEM_E_BUSY:
                ymodem_putc(CA);
                ymodem_putc(CA);
                return YMODEM_E_BUSY;

            default:
                if (session_begin > 0)
                {
                    errors++;
                }
                if (errors > PKT_TRY_TIMES)
                {
                    ymodem_putc(CA);
                    ymodem_putc(CA);
                    return YMODEM_E_RECV;
                }
                ymodem_putc(CRC16);
                break;
            }
            if (file_done != 0)
            {
                break;
            }
        }
        if (session_done != 0)
        {
            break;
        }
    }
    write_offset = 0;
    return YMODEM_OK;
}

#pragma region ymodem_update
extern QueueHandle_t g_rx_ymodem_q;
static void uart4_send(uint8_t cChar)
{
    usart_data_transmit(UART4, (uint8_t)cChar);
    while (RESET == usart_flag_get(UART4, USART_FLAG_TBE))
        ;
}

static int uart4_recv(void)
{
    int rx_data = -1;
    uint32_t timeout = 0x200000;
    while (timeout-- > 1)
    {
        if (usart_flag_get(UART4, USART_FLAG_RBNE) != RESET)
        {
            rx_data = (uint8_t)usart_data_receive(UART4);
            return (char)rx_data;
        }
    }
    return -1;
}

static void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < (ms * 1000000); i++)
    {
        __NOP();
    }
}

extern FRESULT fatfs_read_data(const char *path, uint32_t offset, void *buffer, uint32_t len);
TaskHandle_t g_ymodem_handle = NULL;

void ymodem_update_task(void)
{
    static int file_size = 0;
    FRESULT ret;
    // uint8_t ymodem_recv_data[PKT_DATA_1K + 32] = {0};
    uint8_t file_name[FILE_NAME_LENGTH] = {0};
    // char *rx_buffer = NULL;

    ymodem_io_init(uart4_send, uart4_recv, cal_crc16, delay_ms);
    vTaskDelay(500);
    hk_print_msg(HK_INFO, "task will run....");
    hk_print_msg(HK_INFO, "传入文件名称长度不能大于64字节.....");
    hk_print_msg(HK_INFO, "传入文件名称需要特定要求.....");
    while (1)
    {
        taskENTER_CRITICAL();
        int ret = ymodem_file_recvive(file_name, NULL, &file_size); // ymodem recv
        taskEXIT_CRITICAL();
        if (ret == YMODEM_OK)
        {
            fatfs_getfree();
            hk_print_msg(HK_INFO, "file name =%s", file_name);
            hk_print_msg(HK_INFO, "file size =%d b", file_size);

            if (file_size > 0)
            {
                if (strstr(file_name, "mcu-update"))
                {
                    hk_print_msg(HK_INFO, "开始BMC系统升级,请等待...");
                    upgrade_bmc();
                }
                else if (strstr(file_name, "bios-update"))
                {
                    hk_print_msg(HK_INFO, "开始BIOS系统升级,请等待...");
                    gpio_spi_select_upgrade();
                    upgrade_bios();
                    gpio_spi_select_default();
                }
                else
                {
                    hk_print_msg(HK_INFO, "测试文件打印");
                    fatfs_print_data(OTHER_TEST_FILE, file_size);
                    printf("\r\n[升级失败]:错误文件名 \
                            \r\nBIOS文件名:bios-update \
                            \r\nBMC文件名:mcu-update\r\n");
                }
            }

        exit:
            printf("\r\n");
            g_ymodem_handle = NULL;
            hk_print_msg(HK_INFO, "系统升级失败, 重启...");
            NVIC_SystemReset();
            vTaskDelete(NULL);
        }

        vTaskDelay(1);
    }
}

void ymodem_update_thread(void)
{
    if (g_ymodem_handle == NULL)
        xTaskCreate((TaskFunction_t)ymodem_update_task, "ymodem_update_task", 1024 * 2, NULL, 7, &g_ymodem_handle);
    else
        hk_print_msg(HK_DEBUG, "the task is running...");
}
#pragma endregion