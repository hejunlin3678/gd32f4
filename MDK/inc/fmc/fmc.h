#ifndef FMC_H
#define FMC_H

#include "gd32f4xx.h"

#define     FMC_TEST_ADDRESS    (0x082C0000ul)
#define     FMC_IP_ADDRESS    ((0x082C0000ul)+(0x8ul))

//FLASH写入操作
fmc_state_enum flash_write(uint32_t addr, uint32_t dwLen, uint8_t *buff);

//FLASH读取操作
uint32_t flash_read(uint32_t addr, uint32_t len, uint8_t *buff);

//FLASH擦除操作
fmc_state_enum flash_erase_sector(uint32_t addr_start, uint32_t addr_end);

fmc_state_enum flash_write_byte( uint32_t address, uint8_t len, uint8_t *pdata);

#endif /* ifndef FMC_H */
