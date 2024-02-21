#include "gd32f4xx.h"
#include "systick.h"
#include "spi/gd25qxx.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

// #define BUFFER_SIZE              256
// #define SFLASH_ID                0xC84015
// #define SFLASH_ID_GD             0xC84013
// #define FLASH_WRITE_ADDRESS      0x000000
// #define FLASH_READ_ADDRESS       FLASH_WRITE_ADDRESS


#if 0 

void upgrade_bios_true(void)
{

    uint8_t tx_buffer[BUFFER_SIZE];
    uint8_t rx_buffer[BUFFER_SIZE];
    uint32_t flash_id = 0;
    uint16_t i = 0;
    uint8_t  is_successful = 0;

    /* get flash id */
    flash_id = spi_flash_read_id();
    printf("\n\rThe Flash_ID:0x%X\n\r\n\r",flash_id);
    /* flash id is correct */
    if((SFLASH_ID_GD == flash_id)||(SFLASH_ID == flash_id)){
        printf("\n\rWrite to tx_buffer:\n\r\n\r");

        /* printf tx_buffer value */
        for(i = 0; i < BUFFER_SIZE; i++){
            tx_buffer[i] = i;
            printf("0x%02X ",tx_buffer[i]);

            if(15 == i%16)
                printf("\n\r");
        }
        /* erase the specified flash sector */
        spi_flash_sector_erase(FLASH_WRITE_ADDRESS);

        /* write tx_buffer data to the flash */ 
        qspi_flash_buffer_write(tx_buffer,FLASH_WRITE_ADDRESS,BUFFER_SIZE);
        
        vTaskDelay(10);
        /* read a block of data from the flash to rx_buffer */
        qspi_flash_buffer_read(rx_buffer,FLASH_READ_ADDRESS,BUFFER_SIZE);

        /* printf rx_buffer value */
        for(i = 0; i < BUFFER_SIZE; i ++){
            printf("0x%02X ", rx_buffer[i]);
            if(15 == i%16)
                printf("\n\r");
        }

        /* spi qspi flash test passed */
        if(0 == is_successful){
            printf("\n\rSPI-GD25Q40 Test Passed!\n\r");
        }
    }else{
        /* spi flash read id fail */
        printf("\n\rSPI Flash: Read ID Fail!\n\r");
    }
}

#endif
