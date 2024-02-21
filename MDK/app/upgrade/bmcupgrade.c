#include <stdio.h>
#include <string.h>
#include "gd32f4xx.h"
#include "upgrade/bootload.h"
#include "fatfs/fatfs_user.h"
#include "fmc/fmc.h"
#include "spi/gd25qxx.h"
#include "systick.h"
#include "hk_common.h"

#define BUFFER_SIZE 256
#define SFLASH_ID 0xC84015
#define SFLASH_ID_GD 0xC84013
#define FLASH_WRITE_ADDRESS 0x000000
#define FLASH_READ_ADDRESS FLASH_WRITE_ADDRESS

struct hexStruct
{
    uint8_t len;
    uint8_t addr[2];
    uint8_t type;
    uint8_t data[16];
    uint8_t checkSum;
};

static update_state_enum g_update_who = UPDATE_OK;

static uint8_t upgrade_flag = 0;

static void printHexStruct(struct hexStruct *raw)
{
    if (dbg_level >= HK_DEBUG) // 调试等级打印
    {
        printf(":%02X", raw->len);
        printf("%02X%02X", raw->addr[0], raw->addr[1]);
        printf("%02X", raw->type);
        for (int i = 0; i < raw->len; i++)
            printf("%02X", raw->data[i]);

        printf("%02X\r\n", raw->checkSum);
    }
    else
    {
    }
}

static void delay_1ms(uint32_t ms)
{
    vTaskDelay(ms);
}

static uint8_t char2uint(const char *input, uint8_t *output)
{
    for (int i = 0; i < strlen(input); i++)
    {
        output[i] &= 0x00;
        for (int j = 1; j >= 0; j--)
        {
            char hb = input[i * 2 + 1 - j];
            if (hb >= '0' && hb <= '9')
            {
                output[i] |= (uint8_t)((hb - '0') << (4 * j));
            }
            else if (hb >= 'a' && hb <= 'f')
            {
                output[i] |= (int8_t)((hb - 'a' + 10) << (4 * j));
            }
            else if (hb >= 'A' && hb <= 'F')
            {
                output[i] |= (int8_t)((hb - 'A' + 10) << (4 * j));
            }
            else if (hb == '\0')
                return 0;
        }
    }
    return 1;
}

static update_state_enum bios_update(struct hexStruct *raw)
{
    static uint32_t baseAddr = 0x0000;
    uint8_t data_test[32] = {0};
    printHexStruct(raw);

    if (raw->type == 0x04)
    {
        baseAddr = ((raw->data[0] << 8) + (raw->data[1])) << 16;
        printf("baseAddr=[0x%X]\r\n", baseAddr);
    }

    if (raw->type == 0x00)
    {
        uint32_t address = baseAddr + (raw->addr[0] << 8) + raw->addr[1];
        // printf("FMC-raw: addr[0x%2X] len[0x%X]\r\n", address, raw->len);
        spi_flash_buffer_write(raw->data, address, raw->len);
        delay_1ms(1U);
    }
    return UPDATE_OK;
}

static update_state_enum arm_update(struct hexStruct *raw)
{
    static uint32_t baseAddr = 0x0000;
    static uint8_t erase_flg = 0;

    printHexStruct(raw);

    //type:升级文件类型 
    if (raw->type == 0x04)  //升级地址
    {
        baseAddr = ((raw->data[0] << 8) + (raw->data[1])) << 16;

        if ((IS_APP_RUN && (baseAddr == BOOT_APP_FLASH_START_ADDR)) ||
            (IS_APP_RUN_BK && (baseAddr == BOOT_APP_FLASH_BK_START_ADDR)) ||
            (IS_UPLOAD && (baseAddr == BOOT_APP_FLASH_START_ADDR)))
        {
            printf("ERROR NO UPDATE CURRENT SECTOR [0x%X]\r\n", baseAddr);
            return UPDATE_FAIL;
        }

        if (erase_flg == 0)
        {
            printf("flash_erase_sector [%04X][%04X]\r\n", baseAddr, (baseAddr + 0x10000));
            fmc_state_enum tRtn = flash_erase_sector(baseAddr, (baseAddr + 0x10000));
            if (FMC_READY != tRtn)
            {
                printf("ERROR flash_erase_sector [%d]\r\n", tRtn);
                return UPDATE_FAIL;
            }
        }

        if (baseAddr == BOOT_APP_FLASH_START_ADDR)
        {
            g_update_who = UPDATE_APP;
        }
            

        if (baseAddr == BOOT_APP_FLASH_BK_START_ADDR)
        {
            g_update_who = UPDATE_APP_BK;
        }

        erase_flg = 1;

        printf("baseAddr=[0x%X] g_update_who=[%d]\r\n", baseAddr, g_update_who);
    }

    if (raw->type == 0x00)  //升级数据
    {
        uint32_t address = baseAddr + (raw->addr[0] << 8) + raw->addr[1];
        // printf("FMC-raw: addr[0x%2X] len[0x%X]\r\n", address, raw->len);
        fmc_state_enum tRtn = flash_write_byte(address, raw->len, raw->data);
        if (FMC_READY != tRtn)
        {
            printf("ERROR arm_update [%d] !!!\r\n", tRtn);
            return UPDATE_FAIL;
        }
    }

    if (raw->type == 0x01)  //升级结束
    {
        // printf("upgrade_flag=3 \r\n");
        upgrade_flag = 3;
    }

    delay_1ms(1U);

    return UPDATE_OK;
}

static uint8_t get_upload_bmc_data(const char *data, struct hexStruct *raw)
{
    uint16_t sum = 0x00;
    uint8_t tmp[22] = {0};
    uint8_t i = 0, j = 4;
    uint32_t baseAddr_check = 0x0000;

    char2uint(data, tmp);
    raw->len = tmp[0];
    raw->addr[0] = tmp[1];
    raw->addr[1] = tmp[2];
    raw->type = tmp[3];
    sum = tmp[0] + tmp[1] + tmp[2] + tmp[3];

    if (raw->len == 16)
        if (strlen(data) < 42)
            return 0;

    for (i = 0; (i < raw->len) & (j < (strlen(data) / 2) - 1); i++, j++)
    {
        // printf("tmp[%d]=[0x%X]\r\n", j, tmp[j]);
        raw->data[i] = tmp[j];
        sum += raw->data[i];
    }

    raw->checkSum = tmp[j];

    if (((0x100 - sum) & 0xFF) != raw->checkSum)
        return 0;

    if(upgrade_flag)
        return 1;
    // 0x08020000 // 0x08060000
    if (raw->type == 0x04)
    {
        baseAddr_check = ((raw->data[0] << 8) + (raw->data[1])) << 16;

        if ((IS_APP_RUN && (baseAddr_check == BOOT_APP_FLASH_START_ADDR)) ||
            (IS_APP_RUN_BK && (baseAddr_check == BOOT_APP_FLASH_BK_START_ADDR)) ||
            (IS_UPLOAD && (baseAddr_check == BOOT_APP_FLASH_START_ADDR)))
        {
            printf("search addr [0x%X] , waiting next ...\r\n", baseAddr_check);
            return 2;
        }
        else
        {
            upgrade_flag = 1;
            printf("BMC升级中......请等待......\r\n");
            return 1;
        }     
    }
    else
        return 2;
    
    // else if ((raw->type == 0x04) && (tmp[4] == 0x08) && (tmp[5] == 0x06))
    // {
    //     baseAddr_check = ((raw->data[0] << 8) + (raw->data[1])) << 16;
    //     if ((IS_APP_RUN && (baseAddr_check == BOOT_APP_FLASH_START_ADDR)) ||
    //         (IS_APP_RUN_BK && (baseAddr_check == BOOT_APP_FLASH_BK_START_ADDR)) ||
    //         (IS_UPLOAD && (baseAddr_check == BOOT_APP_FLASH_START_ADDR)))
    //     {
    //         printf("ERROR NO UPDATE CURRENT SECTOR 0x08060000[0x%X]\r\n", baseAddr_check);
    //         return 2;
    //     }
    //     else
    //     {
    //         upgrade_flag = 1;
    //         printf("BMC升级中......请等待......\r\n");
    //         return 1;   
    //     }      
    // }
    // else
    //     return 2;
}

static uint8_t get_upload_data(const char *data, struct hexStruct *raw)
{
    uint16_t sum = 0x00;
    uint8_t tmp[22] = {0};
    uint8_t i = 0, j = 4;

    char2uint(data, tmp);
    raw->len = tmp[0];
    raw->addr[0] = tmp[1];
    raw->addr[1] = tmp[2];
    raw->type = tmp[3];
    sum = tmp[0] + tmp[1] + tmp[2] + tmp[3];

    if (raw->len == 16)
        if (strlen(data) < 42)
            return 0;

    for (i = 0; (i < raw->len) & (j < (strlen(data) / 2) - 1); i++, j++)
    {
        // printf("tmp[%d]=[0x%X]\r\n", j, tmp[j]);
        raw->data[i] = tmp[j];
        sum += raw->data[i];
    }
    // printf("strlen(data)=%d\r\n", strlen(data));
    // printf("tmp[%d]=[0x%X]\r\n", j, tmp[j]);
    raw->checkSum = tmp[j];
    // printf("sum=[0x%X] check=[0x%X]\r\n", ((0x100 - sum) & 0xFF), raw->checkSum);
    if (((0x100 - sum) & 0xFF) != raw->checkSum)
    {
        // printf("\r\nsum=[0x%X] check=[0x%X]\r\n", ((0x100 - sum) & 0xFF), raw->checkSum);
        return 0;
    }

    return 1;
}

static update_state_enum write_update_flg(void)
{
    uint8_t appFlg[] = {0xA5, 0x5A, 0x5A, 0xA5};
    uint8_t appBkFlg[] = {0x5A, 0xA5, 0xA5, 0x5A};

    printf("flash_erase_sector [%04X][%04X]\r\n", BOOT_UPDATE_OPT_ADDR, (BOOT_UPDATE_OPT_ADDR + 0x4));
    fmc_state_enum tRtn = flash_erase_sector(BOOT_UPDATE_OPT_ADDR, (BOOT_UPDATE_OPT_ADDR + 0x4));
    if (FMC_READY != tRtn)
    {
        printf("ERROR flash_erase_sector [%d]\r\n", tRtn);
        return UPDATE_FAIL;
    }

    if (g_update_who == UPDATE_APP)
    {
    update_app_flg:
        printf("Write UPDATE_APP flag to fmc!\r\n");

        if (FMC_READY != flash_write_byte(BOOT_UPDATE_OPT_ADDR, 4, appFlg))
            printf("ERROR UPDATE_APP [%d] !!!\r\n", g_update_who);
        printf("Update_Flg:0x%X\r\n", BOOT_UPDATE_OPT_APP_FLAG);

        if (IS_APP_RUN == 0)
            goto update_app_flg;
    }

    if (g_update_who == UPDATE_APP_BK)
    {
    update_app_bk_flg:
        printf("Write UPDATE_APP_BK flag to fmc!\r\n");

        if (FMC_READY != flash_write_byte(BOOT_UPDATE_OPT_ADDR, 4, appBkFlg))
            printf("ERROR UPDATE_APP_BK [%d] !!!\r\n", g_update_who);
        printf("Update_Flg:0x%X\r\n", BOOT_UPDATE_OPT_APP_FLAG);

        if (IS_APP_RUN_BK == 0)
            goto update_app_bk_flg;
    }

    // if (f_unlink(UPLOAD_BMC_FILE) == FR_OK)  //ymodem升级传入时删除对应升级文件
    //     printf("succeed to delete file %s ! \r\n", UPLOAD_BMC_FILE);

    return UPDATE_OK;
}

update_state_enum http_upgrade_bmc(char *pbuf)
{
    static char fdatas_prev[128];
    update_state_enum state = UPDATE_OK;
    static struct hexStruct hexData;
    uint8_t lineCount = 0;
    uint8_t upFlg = 1;

    char *fdatas = strtok(pbuf, "\r\n");

    while (fdatas != NULL)
    {
        // printf( "%d:[%s]\r\n", lineCount, fdatas );

        if (lineCount >= 4 && fdatas[0] != '-')
        {
            // printf("===000=== [%s]\r\n", fdatas);
            if (fdatas[0] != ':')
            {
                strcat(fdatas_prev, fdatas);
                // printf("===111=== [%s]\r\n", fdatas_prev);
                upFlg = get_upload_data(&fdatas_prev[1], &hexData);
                if (upFlg)
                {
                    // printHexStruct(&hexData);
                    state = arm_update(&hexData);
                    if (UPDATE_FAIL == state)
                        break;

                    memset(fdatas_prev, 0, sizeof(fdatas_prev));
                }
                else
                { // 特殊情况
                    if (fdatas[0] != '0')
                    {
                        printf("strlen(fdatas)=[%d][%s]", strlen(fdatas), fdatas_prev);
                        state = UPDATE_FAIL;
                        break;
                    }
                }
            }
            else
            {
                upFlg = get_upload_data(&fdatas[1], &hexData);
                if (upFlg)
                {
                    // printf("===222=== [%s]\r\n", fdatas);
                    // printHexStruct(&hexData);
                    state = arm_update(&hexData);
                    if (UPDATE_FAIL == state)
                        break;
                }
                else
                {
                    // printf("===333=== [%s]\r\n", fdatas);
                    memset(fdatas_prev, 0, sizeof(fdatas_prev));
                    memcpy(fdatas_prev, fdatas, strlen(fdatas));
                    // strcpy(fdatas_prev, fdatas);
                }
            }
        }

        fdatas = strtok(NULL, "\r\n");
        lineCount++;
    }

    return state;
}

void http_merge_bmc(update_state_enum state)
{
    if (UPDATE_FAIL != state)
    {
        if (UPDATE_OK == write_update_flg())
        {
            printf("UPDATE SUCESS ! [%d][%d]\r\n\r\n", g_update_who, state);
            NVIC_SystemReset();
        }
    }
    else
    {
        printf("UPDATE FAILED ! [%d]\r\n", state);
    }
}

update_state_enum http_upgrade_bios(char *pbuf)
{
    static char fdatas_prev[128];
    update_state_enum state = UPDATE_OK;
    static struct hexStruct hexData;
    uint8_t lineCount = 0;
    uint8_t upFlg = 1;

    char *fdatas = strtok(pbuf, "\r\n");

    while (fdatas != NULL)
    {
        // printf( "%d:[%s]\r\n", lineCount, fdatas );

        if (lineCount >= 4 && fdatas[0] != '-')
        {
            // printf("===000=== [%s]\r\n", fdatas);
            if (fdatas[0] != ':')
            {
                strcat(fdatas_prev, fdatas);
                // printf("===111=== [%s]\r\n", fdatas_prev);
                upFlg = get_upload_data(&fdatas_prev[1], &hexData);
                if (upFlg)
                {
                    // printHexStruct(&hexData);
                    state = bios_update(&hexData);
                    if (UPDATE_FAIL == state)
                        break;

                    memset(fdatas_prev, 0, sizeof(fdatas_prev));
                }
                else
                { // 特殊情况
                    if (fdatas[0] != '0')
                    {
                        printf("strlen(fdatas)=[%d][%s]", strlen(fdatas), fdatas_prev);
                        state = UPDATE_FAIL;
                        break;
                    }
                }
            }
            else
            {
                upFlg = get_upload_data(&fdatas[1], &hexData);
                if (upFlg)
                {
                    // printf("===222=== [%s]\r\n", fdatas);
                    // printHexStruct(&hexData);
                    state = bios_update(&hexData);
                    if (UPDATE_FAIL == state)
                        break;
                }
                else
                {
                    // printf("===333=== [%s]\r\n", fdatas);
                    memset(fdatas_prev, 0, sizeof(fdatas_prev));

                    // strcpy(fdatas_prev, fdatas);
                    memcpy(fdatas_prev, fdatas, strlen(fdatas));
                }
            }
        }

        fdatas = strtok(NULL, "\r\n");
        lineCount++;
    }

    return state;
}



void upgrade_bmc(void)
{
    FIL fil;    /* File object */
    FRESULT fr; /* FatFs return code */
    update_state_enum state = UPDATE_OK;
    static struct hexStruct hexData;
    static char fdatas_prev[64];
    static char fdatas[64];
    int tempLocate;
    uint8_t upFlg = 1;
    int readsize = 64;

    memset(fdatas, 0, sizeof(fdatas));

    fr = f_open(&fil, UPLOAD_BMC_FILE, FA_READ);
    if (fr != FR_OK)
    {
        printf("f_open [%s] failed!\r\n", UPLOAD_BMC_FILE);
        goto error;
    }
    f_rewind(&fil);
    tempLocate = f_tell(&fil);
    while (tempLocate < fil.fsize)
    {
        f_gets(fdatas, readsize, &fil);

        if(upgrade_flag == 3)
        {
            fil.fsize = 0;
            goto exit;
        }

        if (fdatas[0] != ':')
        {
            strcat(fdatas_prev, fdatas);
            // printf("===111=== [%s]\r\n", fdatas_prev);
            upFlg = get_upload_bmc_data(&fdatas_prev[1], &hexData);
            if(upFlg == 2)
                goto exit;

            if (upFlg == 1)
            {
                // printHexStruct(&hexData);
                state = arm_update(&hexData);
                if (UPDATE_FAIL == state)
                    break;

                memset(fdatas_prev, 0, sizeof(fdatas_prev));
            }
            else
            { // 特殊情况
                if (fdatas[0] != '0')
                {
                    printf("strlen(fdatas)=[%d][%s]", strlen(fdatas), fdatas_prev);
                    state = UPDATE_FAIL;
                    break;
                }
            }
        }
        else
        {
            upFlg = get_upload_bmc_data(&fdatas[1], &hexData);
            if(upFlg == 2)
                goto exit;
            if (upFlg)
            {
                // printf("===222=== [%s]\r\n", fdatas);
                // printHexStruct(&hexData);
                state = arm_update(&hexData);
                if (UPDATE_FAIL == state)
                    break;
            }
            else
            {
                // printf("===333=== [%s]\r\n", fdatas);
                memset(fdatas_prev, 0, sizeof(fdatas_prev));

                // strcpy(fdatas_prev, fdatas);  //buffer出现过溢出？
                memcpy(fdatas_prev, fdatas, strlen(fdatas));
            }
        }

    exit:
        tempLocate = f_tell(&fil);

        f_lseek(&fil, tempLocate);
        memset(fdatas, 0, sizeof(fdatas));

    }

    f_sync(&fil);
    f_close(&fil);

error:
    if (UPDATE_FAIL != state)
    {
        if (UPDATE_OK == write_update_flg())
        {
            printf("BMC UPDATE SUCESS ! [%d][%d]\r\n\r\n", g_update_who, state);
            hk_print_msg(HK_INFO, "BMC升级成功,系统即将重启...\r\n");
            NVIC_SystemReset();
        }
    }
    else
    {
        printf("BMC UPDATE FAILED ! [%d]\r\n", state);
    }
}

/* TODO: Change */
void upgrade_bios(void)
{
    FIL fil;    /* File object */
    FRESULT fr; /* FatFs return code */
    update_state_enum state = UPDATE_OK;
    static struct hexStruct hexData;
    static char fdatas_prev[64];
    static char fdatas[64];
    int tempLocate;
    uint8_t upFlg = 1;
    int readsize = 64;
    uint32_t flash_id = 0;

    /* get flash id */
    flash_id = spi_flash_read_id();
    if ((flash_id == 0xFFFFFF) || (flash_id == 0))
    {
        printf("\n\rERROR : The Flash_ID:0x%X\n\r\n\r", flash_id);
        state = UPDATE_FAIL;
        goto error;
    }

    memset(fdatas, 0, sizeof(fdatas));
    fr = f_open(&fil, UPLOAD_BIOS_FILE, FA_READ);
    if (fr != FR_OK)
    {
        printf("f_open [%s] failed!\r\n", UPLOAD_BIOS_FILE);
        state = UPDATE_FAIL;
        goto error;
    }

    spi_flash_bulk_erase();
    spi_flash_wait_for_eraes_end();

    f_rewind(&fil);
    tempLocate = f_tell(&fil);
    while (tempLocate < fil.fsize)
    {
        f_gets(fdatas, readsize, &fil);

        if (fdatas[0] != '-' && fdatas[1] != 'o' &&
            fdatas[0] != '\r' && fdatas[0] != '\n' &&
            fdatas[0] != '\0' && fdatas[0] != ';')
        {
            if (strlen(fdatas) < 5)
                printf("strlen(fdatas)=%d [%s]\r\n", strlen(fdatas), fdatas);
            fdatas[strlen(fdatas) - 2] = '\0';
            if (fdatas[0] != ':')
            {
                strcat(fdatas_prev, fdatas);
                upFlg = get_upload_data(&fdatas_prev[1], &hexData);
                if (upFlg)
                {
                    state = bios_update(&hexData);
                    if (UPDATE_FAIL == state)
                    {
                        printf("update: error.....\r\n");
                        break;
                    }
                    memset(fdatas_prev, 0, sizeof(fdatas_prev));
                }
                else
                { // 特殊情况
                    if (fdatas[0] != '0')
                    {
                        printf("strlen(fdatas)=[%d]fdatas_prev=[%s]fdatas=[%s]\r\n", strlen(fdatas), fdatas_prev, fdatas);
                        state = UPDATE_FAIL;
                        break;
                    }
                }
            }
            else
            {
                upFlg = get_upload_data(&fdatas[1], &hexData);
                if (upFlg)
                {
                    state = bios_update(&hexData);
                    if (UPDATE_FAIL == state)
                    {
                        printf("update error.....\r\n");
                        break;
                    }
                }
                else
                {
                    // printf("===333=== [%s]\r\n", fdatas);
                    memset(fdatas_prev, 0, sizeof(fdatas_prev));
                    memcpy(fdatas_prev, fdatas, 64);
                    // strcpy(fdatas_prev, fdatas); //todo+
                }
            }
        }

        tempLocate = f_tell(&fil);

        f_lseek(&fil, tempLocate);
        memset(fdatas, 0, sizeof(fdatas));
    }

    f_sync(&fil);
    f_close(&fil);

error:
    if (UPDATE_FAIL != state)
    {
        // if ( UPDATE_OK == write_update_flg() )  //bios升级不需要
        {
            printf("BIOS UPDATE SUCESS ! [%d][%d]\r\n\r\n", g_update_who, state);
            printf("BIOS升级成功,系统即将重启...\r\n");

            bc_cpu_power_off();
            vTaskDelay(1000);
            NVIC_SystemReset();
        }
    }
    else
    {
        printf("BIOS UPDATE FAILED ! [%d]\r\n", state);
        printf("系统升级失败 !!!\r\n\r\n");
    }
}
