#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "gd32f4xx.h"
#include "emmc/emmc.h"
#include "hk_common.h"
#include "fatfs/ff.h"
#include "fatfs/diskio.h"
#include "fatfs/fatfs_user.h"
#include "wdg/hk_wdg.h"
static FATFS g_fatfs_fs;
static FIL g_fatfs_fil;

const char *SDPath = "0:/";
uint32_t totalSize = 0;
uint32_t freeSize = 0;

// system_data_t g_sys_data = {0};

FRESULT fatfs_init(void)
{
    uint16_t stat = 0;
    uint16_t i = 5;
    do
    {
        stat = disk_initialize(0);
        // printf("\r\nf_mkfs init.... [%d]\r\n", stat);
    } while ((stat != 0) && (--i));

    stat = f_mount(&g_fatfs_fs, SDPath, 1);
    if (stat != FR_OK)
    {
        printf("\r\nf_mount Failed => [%d]\r\n", stat);
        if (stat == FR_NO_FILESYSTEM)
        {
            stat = f_mkfs(SDPath, 0, 4096);
            if (stat != FR_OK)
            {
                printf("\r\nf_mkfs Failed => [%d]\r\n", stat);
            }
        }
        return FR_OK;
    }
    // printf("\r\nf_mkfs ok => [%d]\r\n", stat);
    return FR_OK;
}

FRESULT fatfs_getfree(void)
{
    uint16_t stat = 0;
    FATFS *fs1 = NULL;
    uint32_t fre_clust = 0, fre_sect = 0, tot_sect = 0;

    stat = f_getfree((const TCHAR *)SDPath, (DWORD *)&fre_clust, &fs1);
    if (stat == FR_OK)
    {
        tot_sect = (fs1->n_fatent - 2) * fs1->csize; //得到总扇区数
        fre_sect = fre_clust * fs1->csize;           //得到空闲扇区数
#if _MAX_SS != 512                                   //扇区大小不是512字节,则转换为512字节
        tot_sect *= fs1->ssize / 512;
        fre_sect *= fs1->ssize / 512;
#endif
        totalSize = tot_sect >> 1; //单位为KB
        freeSize = fre_sect >> 1;  //单位为KB
        hk_print_msg(HK_INFO, "EMMC TotalSize:%d Kb;freeSize: %d Kb", totalSize, freeSize);
    }

    if (stat == FR_NO_FILESYSTEM)
    {
        stat = f_mkfs((const TCHAR *)SDPath, 0, 0);
        printf("\r\nf_mkfs -> stat=[0x%x]\r\n", stat);
    }

    return FR_OK;
}

FRESULT fatfs_open_append(const char *path)
{
    FRESULT fr = FR_OK;
    /* Opens an existing file. If not exist, creates a new file. */
    fr = f_open(&g_fatfs_fil, path, FA_WRITE | FA_OPEN_ALWAYS);
    if (fr == FR_OK)
    {
        /* Seek to end of the file to append data */
        fr = f_lseek(&g_fatfs_fil, f_size(&g_fatfs_fil));
        //  fr = f_lseek(&g_fatfs_fil, 0);
        if (fr != FR_OK)
            f_close(&g_fatfs_fil);
    }
    return fr;
}

FRESULT fatfs_read(const char *path, void *buffer, uint8_t len)
{
    FRESULT fr; /* FatFs return code */
    char *line = NULL;
    line = (char *)malloc(sizeof(char) * len); /* Line buffer */

    /* Open a text file */
    fr = f_open(&g_fatfs_fil, path, FA_READ);
    if (fr)
        return fr;

    /* Read every line and display it */
    while (f_gets(line, len, &g_fatfs_fil))
    {
        strcpy(buffer, line);
    }

    /* Close the file */
    f_close(&g_fatfs_fil);
    if (line)
        free(line);
    return fr;
}

FRESULT fatfs_write(void *buffer)
{
    FRESULT fr;
    // /* Open or create a log file and ready to append */
    // fr = open_append(&g_fatfs_fil, path);
    // if (fr != FR_OK) return FR_INVALID_PARAMETER;

    /* Append a line */
    f_printf(&g_fatfs_fil, "%s", buffer);

    /* Sync file ops */
    fr = f_sync(&g_fatfs_fil);

    // /* Close the file */
    // f_close(&g_fatfs_fil);

    return fr;
}

FRESULT fatfs_test(FatfsOps *ops)
{
    FIL openFile;
    uint16_t result = 0;
    UINT br, bw;

    printf("\r\nOpen File [%s]\r\n", ops->path);
    if (ops->rw)
        printf("\r\nFATFS File Write:[%s]\r\n", ops->buffer);

    result = f_open(&openFile, (const TCHAR *)ops->path, FA_CREATE_NEW | FA_WRITE);
    if ((FR_OK == result) || (FR_EXIST == result))
    {
        if (FR_EXIST == result)
        {
            if (ops->rw)
            {
                f_printf(&openFile, "%s", ops->buffer);
                f_sync(&openFile);
                printf("\r\nFATFS File Write Success!\r\n");
            }
            else
            {
                f_open(&openFile, (const TCHAR *)ops->path, FA_OPEN_EXISTING | FA_READ);
                memset(ops->buffer, '\0', sizeof(ops->buffer));
                result = f_read(&openFile, ops->buffer, 512u, &br);
                printf("\r\nFATFS File Content:[%s]\r\n", ops->buffer);
            }
        }
        else
        {
            f_sync(&openFile);
            if (ops->rw)
            {
                result = f_write(&openFile, ops->buffer, sizeof(ops->buffer), &bw);
                printf("\r\nFATFS File Write Success!\r\n");
            }
            else
                printf("\r\nFATFS File Not Exist!\r\n");
        }
    }
    else if (FR_NO_FILESYSTEM == result)
    {
        printf("\r\nPlease Format first!\r\n");
    }
    else
    {
        printf("\r\nOpen File Failed!\r\n");
    }

    f_close(&openFile);
    return FR_OK;
}

FRESULT fatfs_close(void)
{
    FRESULT fr = FR_OK;
    // if (&g_fatfs_fil)
    fr = f_close(&g_fatfs_fil);
    return fr;
}

//文件删除
FRESULT fatfs_file_delete(const char *path)
{
    FRESULT fr = FR_INT_ERR;
    // if (&g_fatfs_fil)
    {
        fr = f_unlink(path);
        if (fr == FR_OK)
            hk_print_msg(HK_DEBUG, "file delete ok...");
    }
    return fr;
}

// 数据写入
FRESULT fatfs_write_data(char *path, uint8_t offset_en, uint32_t offset, void *buffer, uint32_t buffer_len)
{
    FRESULT fr;
    uint32_t bw;
    /* Open or create a log file and ready to append */
    /* Open a text file */

    fr = f_open(&g_fatfs_fil, path, FA_WRITE | FA_OPEN_ALWAYS);
    // fr = f_open(&g_fatfs_fil, path, FA_CREATE_NEW | FA_WRITE);
    if (fr != FR_OK)
    {
        hk_print_msg(HK_ERROR, "open fail!!!");
        return fr;
    }

    if (offset_en == 0)
        fr = f_lseek(&g_fatfs_fil, f_size(&g_fatfs_fil));
    else
        fr = f_lseek(&g_fatfs_fil, offset);

    if (fr != FR_OK)
    {
        f_close(&g_fatfs_fil);
        hk_print_msg(HK_ERROR, "seek sys data  fail!!!");
        return fr;
    }

    // /* Append a line */
    // fr = fatfs_write(buffer);

    // uint32_t i = 0;
    // while (i++ < buffer_len)
    // {
    //     fr = f_write(&g_fatfs_fil, ((char *)buffer + i), 1, &bw);
    //     if (fr != FR_OK)
    //     {
    //         hk_print_msg(HK_ERROR, "write sys data  fail!!!");
    //         return fr;
    //     }
    // }
    fr = f_write(&g_fatfs_fil, buffer, buffer_len, &bw);
    if (fr != FR_OK)
    {
        hk_print_msg(HK_ERROR, "write sys data  fail!!!");
        return fr;
    }
    /* Sync file ops */

    f_sync(&g_fatfs_fil);

    /* Close the file */
    f_close(&g_fatfs_fil);

    return fr;
}

// 数据读取
FRESULT fatfs_read_data(const char *path, uint32_t offset, void *buffer, uint32_t len)
{
    FRESULT fr; /* FatFs return code */
    uint32_t br;

    /* Open a text file */
    fr = f_open(&g_fatfs_fil, path, FA_READ);
    if (fr)
        return fr;

    f_lseek(&g_fatfs_fil, offset);
    /* Read every line and display it */
    fr = f_read(&g_fatfs_fil, buffer, len, &br);

    /* Close the file */
    f_close(&g_fatfs_fil);

    return fr;
}

// 文件数据打印
FRESULT fatfs_print_data(const char *path, uint32_t file_size)
{
#define READ_DATA_SIZE 512
    FRESULT ret; /* FatFs return code */
    uint32_t br;
    int try_times = 10;
    int read_len;
    char rx_buffer[READ_DATA_SIZE + 1] = {0};
    ret = f_open(&g_fatfs_fil, path, FA_READ);
    if (ret)
    {
        hk_print_msg(HK_ERROR, "open file fail...");
        goto exit;
    }

    for (uint32_t i = 0; i < file_size; i += READ_DATA_SIZE)
    {
        /* Read every line and display it */
        ret = FR_INT_ERR;
        try_times = 10;
        while ((ret != FR_OK) && (try_times > 1))
        {
            try_times--;
            ret = f_lseek(&g_fatfs_fil, i);
            if (ret != FR_OK)
                continue;
            if ((i + READ_DATA_SIZE) > file_size) //处理最后一包读取
                read_len = file_size - i;
            else
                read_len = READ_DATA_SIZE;

            if (read_len > READ_DATA_SIZE)
                read_len = READ_DATA_SIZE;
            memset(rx_buffer, 0, READ_DATA_SIZE);
            IWDG_FEED(); //接收时喂狗
            ret = f_read(&g_fatfs_fil, rx_buffer, read_len, &br);
            rx_buffer[READ_DATA_SIZE] = '\0';
        }
        if (ret == FR_OK)
        {
            printf("%s", rx_buffer);
        }
        else
        {
            hk_print_msg(HK_ERROR, "fatfs_read_data result: %d-->%d b", ret, i);
            break;
        }
    }
    f_sync(&g_fatfs_fil);
    /* Close the file */
    f_close(&g_fatfs_fil);
    return FR_OK;
exit:
    return FR_INT_ERR;
}

#ifdef FAFTS_FUNC_TEST

#define TEST_SIZE 2048
uint16_t write_fg = TEST_SIZE - 1;
uint16_t read_fg = TEST_SIZE - 1;

char tx_buffer[TEST_SIZE] = {0};
char rx_buffer[TEST_SIZE] = {0};

void fafts_test_task(void)
{
    FRESULT ret;
    char *my_path = "0:/111111.TXT";

    vTaskDelay(3000);

    for (uint16_t i = 0; i < TEST_SIZE; i++)
        tx_buffer[i] = '1' + i % 9;

    while (1)
    {
        if (f_unlink(my_path) == FR_OK)
            printf("succeed to delete file %s ! \r\n", my_path);
        memset(rx_buffer, 0, TEST_SIZE);
        vTaskDelay(1000);
        fatfs_getfree();
        ret = fatfs_write_data(my_path, 0, 0, tx_buffer, write_fg);
        if (ret != FR_OK)
        {
            hk_print_msg(HK_ERROR, "!!!!!!!!!!!!!!");
        }
        vTaskDelay(2000);
        ret = fatfs_read_data(my_path, 0, rx_buffer, read_fg);
        if (ret == FR_OK)
        {
            hk_print_msg(HK_DEBUG, "rx_buffer = %s", rx_buffer);
        }
        hk_print_msg(HK_DEBUG, "====OVER====");
        vTaskDelay(1000);
    }

    vTaskDelete(NULL);
}

void fafts_test_thread(void)
{
    xTaskCreate(fafts_test_task, "fafts_test_task", 1024 * 4, NULL, 5, NULL);
}

#endif
