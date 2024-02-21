/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "FreeRTOS.h"
#include "task.h"
#include "fatfs/diskio.h"        /* FatFs lower layer API */
#include "fatfs/ff.h"
#include "gd32f4xx.h"
#include "spi/gd25qxx.h"
#include <stdio.h>

#define ATA                       0     /* Map MMC/SD card to physical drive */
#define SPI_FLASH                 1     /* Map SPI FLASH to physical drive */
#define INTERNAL_FLASH            2     /* Map MCU INTERNAL FLASH to physical drive */

#define BLOCKSIZE                 512   /* block size in bytes */                                              
#define BUSMODE_4BIT                    /* SD 4-bit bus mode, uncommend this macro to choose 1-bit bus mode */
//#define DMA_MODE                        /* SD DMA mode, uncommend this macro to choose polling mode */

#define SPI_FLASH_ID              0xC84015U

#define SPI_FLASH_SECTOR_SIZE     4096
#define SPI_FLASH_SECTOR_NUM      512
#define SPI_FLASH_BLOCK_SIZE      1

#define INTERFLASH_SECTOR_SIZE    2048
#define INTERFLASH_SECTOR_NUM     128   //at least 128
#define INTERFLASH_BLOCK_SIZE     1
#define INTERFLASH_START_ADDRESS  0x08020000

sd_card_info_struct sd_cardinfo;

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
DSTATUS disk_status (
    BYTE pdrv        /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS status = STA_NOINIT;
    
    switch (pdrv) {
        case ATA:    /* SD CARD */
            status = RES_OK;
            break;

        case SPI_FLASH:
            // if(SPI_FLASH_ID == spi_flash_read_id()){
                status = RES_OK;
            // }
            break;
        case INTERNAL_FLASH:
            status = RES_OK;
            break;
        default:
            status = STA_NOINIT;
    }
    return status;
}

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
    BYTE pdrv          /* Physical drive nmuber to identify the drive */
)
{
    DSTATUS status = STA_NOINIT;
    uint32_t cardstate = 0;
    
    switch (pdrv) {
        case ATA:               /* SD CARD */
            /* initialize the card */
            status = sd_init();
            if(MMC_OK == status){
                status = sd_card_information_get(&sd_cardinfo);
            }else{
                return STA_NOINIT;
            }
            if(MMC_OK == status){
                status = sd_card_select_deselect(sd_cardinfo.card_rca);
            }else{
                return STA_NOINIT;
            }
            status = sd_cardstatus_get(&cardstate);
            if(cardstate & 0x02000000){
                /* the card is locked */
                while (1){
                    printf(" ERROR: the card is locked!!!");
                    portYIELD();
                }
            }
            /* configure the bus mode and data transfer mode */
            if(MMC_OK == status){
                /* set bus mode */
                status = sd_bus_mode_config(SDIO_BUSMODE_8BIT);
            }else{
                return STA_NOINIT;
            }
            if(MMC_OK == status){
                /* set data transfer mode */
#ifdef DMA_MODE
                status = sd_transfer_mode_config( MMC_DMA_MODE );
                /* configure the SDIO NVIC */
                nvic_irq_enable(SDIO_IRQn, 0, 0);
#else
                status = sd_transfer_mode_config( MMC_POLLING_MODE );
#endif /* DMA_MODE */
            }else{
                return STA_NOINIT;
            }
            if(MMC_OK == status){
                /* initialize success */
                return 0;
            }else{
                return STA_NOINIT;
            }
    
        case SPI_FLASH:         /* SPI Flash */
            spi_flash_init();
            status = RES_OK;
            break;

        case INTERNAL_FLASH:
            fmc_unlock();
            status = RES_OK;
            break;
        
        default:
            status = STA_NOINIT;
    }
    return status;
}


/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
DRESULT disk_read (
    BYTE pdrv,        /* Physical drive nmuber to identify the drive */
    BYTE *buff,       /* Data buffer to store read data */
    DWORD sector,     /* Start sector in LBA */
    UINT count        /* Number of sectors to read */
)
{
    sd_error_enum status = MMC_ERROR;
    
    /* check the correctness of the parameters */
    if(NULL == buff){
        return RES_PARERR;
    }
    if(!count){
        return RES_PARERR;
    }

    switch (pdrv) {
        case ATA:    /* SD CARD */
            if(1 == count){
                /* single sector read */
                status = sd_block_read((uint32_t *)(&buff[0]), (uint32_t)(sector<<9), BLOCKSIZE);
            }else{
                /* multiple sectors read */
                status = sd_multiblocks_read((uint32_t *)(&buff[0]), (uint32_t)(sector<<9), BLOCKSIZE, (uint32_t)count);
            }
            if(MMC_OK == status){
                return RES_OK;
            }
            return RES_ERROR;
    
        case SPI_FLASH:
            // spi_flash_buffer_read(buff, sector*SPI_FLASH_SECTOR_SIZE, count*SPI_FLASH_SECTOR_SIZE);
            return RES_OK;

        case INTERNAL_FLASH:
            {
                uint8_t *pSource = (uint8_t *)(sector*INTERFLASH_SECTOR_SIZE + INTERFLASH_START_ADDRESS);
                uint16_t i;

                while (count--) {
                    for (i = 0; i < INTERFLASH_SECTOR_SIZE; i++) {
                        *buff++ = *pSource++;
                    }
                }
            }
            return RES_OK;
        
        default:
            return RES_PARERR;
    }
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
#if _USE_WRITE
DRESULT disk_write (
    BYTE pdrv,              /* Physical drive nmuber to identify the drive */
    const BYTE *buff,       /* Data to be written */
    DWORD sector,           /* Start sector in LBA */
    UINT count              /* Number of sectors to write */
)
{
    // uint32_t write_addr; 
    sd_error_enum status = MMC_ERROR;
    if (!count) {
        return RES_PARERR;        /* Check parameter */
    }

    switch (pdrv) {
        case ATA:    /* SD CARD */
            if(1 == count){
                /* single sector write */
                status = sd_block_write((uint32_t *)buff, sector<<9, BLOCKSIZE);
            }else{
                /* multiple sectors write */
                status = sd_multiblocks_write((uint32_t *)buff, sector<<9, BLOCKSIZE, (uint32_t)count);
            }
            if(MMC_OK == status){
                return RES_OK;
            }
            return RES_ERROR;

        case SPI_FLASH:
            // write_addr = sector*SPI_FLASH_SECTOR_SIZE;
            // spi_flash_sector_erase(write_addr);
            // spi_flash_buffer_write((uint8_t *)buff,write_addr,count*SPI_FLASH_SECTOR_SIZE);
            return RES_OK;

        case INTERNAL_FLASH:
            {
                // uint32_t i, page;
                // uint32_t start_page = sector*INTERFLASH_SECTOR_SIZE + INTERFLASH_START_ADDRESS;
                // uint32_t *ptrs = (uint32_t *)buff;

                // page = count;

                // for(; page > 0; page--){
                //     fmc_page_erase(start_page);
                //     i = 0;
                //     do{
                //         fmc_word_program(start_page, *ptrs++);
                //         start_page += 4;
                //     }while(++i < INTERFLASH_SECTOR_SIZE/4);
                // }
            }
            return RES_OK;
        
        default:
            return RES_PARERR;
    }
}
#endif

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
#if _USE_IOCTL
DRESULT disk_ioctl (
    BYTE pdrv,        /* Physical drive nmuber (0..) */
    BYTE cmd,         /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
    uint32_t capacity;
    DRESULT status = RES_PARERR;
    switch (pdrv) {
        case ATA:    /* SD CARD */
            switch (cmd) 
            {
                // Get R/W sector size (WORD) 
                case GET_SECTOR_SIZE :
                    *(WORD * )buff = BLOCKSIZE;
                    break;
                // Get erase block size in unit of sector (DWORD)
                case GET_BLOCK_SIZE :
                    *(DWORD * )buff = sd_cardinfo.card_blocksize;//1;
                    break;

                case GET_SECTOR_COUNT:
                    // *(DWORD * )buff = 1;
                    capacity = sd_card_capacity_get();
                    printf("capacity=[%d]sd_cardinfo.card_blocksize=[%d]\r\n",capacity,sd_cardinfo.card_blocksize);
                    *(DWORD * )buff = capacity*1024/sd_cardinfo.card_blocksize;
                    break;
                case CTRL_SYNC :
                    break;
            }
            return RES_OK;
    
        case SPI_FLASH:
            switch (cmd) {
            /* 512*4096/1024/1024=2(MB), GD25Q16B size: 2MB */
            case GET_SECTOR_COUNT:
                *(DWORD * )buff = SPI_FLASH_SECTOR_NUM;
                break;
            /* sector size */
            case GET_SECTOR_SIZE :
              *(WORD * )buff = SPI_FLASH_SECTOR_SIZE;
                break;
            /* block size */
            case GET_BLOCK_SIZE :
              *(DWORD * )buff = SPI_FLASH_BLOCK_SIZE;
                break;
            }
            return RES_OK;
    
        case INTERNAL_FLASH:
            switch (cmd) {
            case GET_SECTOR_COUNT:
                *(DWORD * )buff = INTERFLASH_SECTOR_NUM;
                break;

            case GET_SECTOR_SIZE :
                *(WORD * )buff = INTERFLASH_SECTOR_SIZE;
                break;

            case GET_BLOCK_SIZE :
                *(DWORD * )buff = INTERFLASH_BLOCK_SIZE;
                break;
            }
            return RES_OK;
        
        default:
            status = RES_PARERR;
    }
    return status;
}
#endif

__weak DWORD get_fattime(void) {
    return    ((DWORD)(2019 - 1980) << 25)    /* Year 2019 */
            | ((DWORD)1 << 21)                /* Month 1 */
            | ((DWORD)1 << 16)                /* Mday 1 */
            | ((DWORD)0 << 11)                /* Hour 0 */
            | ((DWORD)0 << 5)                 /* Min 0 */
            | ((DWORD)0 >> 1);                /* Sec 0 */
}
