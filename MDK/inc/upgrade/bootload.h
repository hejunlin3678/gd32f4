#ifndef _BOOTLOAD_H_
#define _BOOTLOAD_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

#define BAUDRATE_BOOT   (921600)

//V6.01 --2023.12.01修改，硬件改动，修改电流采集算法
#define SOFTWARE_VERSION                ("V6.01")       //莱斯
// #define SOFTWARE_VERSION                ("V6.1")    //32所
#define HARDWARE_VERSION                ("Ver_A")
#define BOOT_APP_FLASH_START_ADDR       (0x08020000U)                /* sector5 128K Byte */
#define BOOT_APP_FLASH_BK_START_ADDR    (0x08060000U)                /* sector8 128K Byte */

#define BOOT_UPDATE_OPT_ADDR            (0x08004000U)                /* sector2 */
#define BOOT_UPDATE_OPT_VALID_TOKEN     0xA55A5AA5U
#define BOOT_UPDATE_BK_OPT_VALID_TOKEN  0x5AA5A55AU
#define BOOT_UPDATE_UPLOAD_VALID_TOKEN  0xFFFFFFFFU

#define BOOT_UPDATE_OPT_APP_FLAG        (*(uint32_t*)(BOOT_UPDATE_OPT_ADDR))
// #define BOOT_UPDATE_OPT_APP_SIZE        (*(int32_t*)(BOOT_UPDATE_OPT_ADDR + 0x00U))
// #define BOOT_UPDATE_OPT_APP_CHECKSUM    (*(uint32_t*)(BOOT_UPDATE_OPT_ADDR + 0x04U))

#define IS_APP_RUN                      (BOOT_UPDATE_OPT_VALID_TOKEN == BOOT_UPDATE_OPT_APP_FLAG)
#define IS_APP_RUN_BK                   (BOOT_UPDATE_BK_OPT_VALID_TOKEN == BOOT_UPDATE_OPT_APP_FLAG)
#define IS_UPLOAD                       (BOOT_UPDATE_UPLOAD_VALID_TOKEN == BOOT_UPDATE_OPT_APP_FLAG)

#define UPLOAD_BMC_FILE 	"0:/0800.HEX"
#define UPLOAD_BIOS_FILE 	"0:/0801.HEX"
#define UPLOAD_CPLD_FILE 	"0:/0802.HEX"

#define OTHER_TEST_FILE 	"0:/TEST.HEX"

#define POWER_OP_NUM     (12)

typedef enum{
    UPDATE_OK,
    UPDATE_APP,
    UPDATE_APP_BK,
    UPDATE_FAIL
}update_state_enum;

typedef enum{
    BOOT_STATE_LOAD,
    BOOT_STATE_JUMP_TO_APP,
    BOOT_STATE_JUMP_TO_APP_BK
}boot_state_enum;


extern SemaphoreHandle_t power_semaphore[POWER_OP_NUM];

typedef  void (*p_function)(void);

void boot_start(void);
update_state_enum http_upgrade_bmc(char *pbuf);
void http_merge_bmc(update_state_enum state);

update_state_enum http_upgrade_bios(char *pbuf);

void upgrade_bmc(void);
void upgrade_bios(void);

#endif /* _BOOTLOAD_H_ */
