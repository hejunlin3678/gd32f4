/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/
/* Library includes. */
#include "gd32f4xx.h"
#include <stdio.h>
/* Application includes. */
#include "fmc/fmc.h"
/*-----------------------------------------------------------*/

#define FLASH_PAGE_SIZE  4

/*=============================================================================
函数名      : getSecNum
功能        : 根据地址得到扇区号
输入参数说明: addr: 地址
返回值说明  : 扇区号
=============================================================================*/
static uint32_t getSecNum (uint32_t addr)
{
	uint8_t 	num=0;
	uint32_t	left, right;

	left = FLASH_BASE;
	right = left + 0x3FFF;

	while (num <= 27)
	{
		// printf("NUM=[%d]\tADDR=[%lx]\tleft=[%lx]\tright=[%lx]\r\n",
		// 		   num, addr, left, right);
		if((addr >= left) && (addr < right))
			break;
		
		++num;
		left = right + 0x01;

		/* 16KB sec: 0 - 3 */
		if (num < 4)
			right = left + 0x3FFF;
		/* 64KB sec: 4 */
		if (num == 4)
			right = left + 0xFFFF;
		/* 128KB sec: 5 - 11 */
		if (num >= 5 && num <= 11)
			right = left + 0x1FFFF;
		/* 16KB sec: 12 - 15 */
		if (num >= 12 && num <= 15)
			right = left + 0x3FFF;
		/* 64KB sec:  16 */
		if (num == 16)
			right = left + 0xFFFF;
		/* 128KB sec:  17 -23 */
		if (num >= 17 && num <= 23)
			right = left + 0x1FFFF;
		/* 256KB sec:  24 -27 */
		if (num >= 24 && num <= 27)
			right = left + 0x3FFFF;
	}

	return num;
		
}
 
/*=============================================================================
函数名      : flash_erase_sector
功能        : 擦除相关扇区
输入参数说明: addr_start: 起始地址
				addr_end: 结束地址
=============================================================================*/
fmc_state_enum flash_erase_sector(uint32_t addr_start, uint32_t addr_end)
{
	uint32_t         start_sec, end_sec;
	uint32_t i = 0;
	
    start_sec = getSecNum(addr_start);
    end_sec   = getSecNum(addr_end);
    
	// printf("start_sec=[%d]\tend_sec=[%d]\r\n", start_sec, end_sec);
	if ( start_sec > 27 )
		return FMC_WPERR;

    fmc_unlock();        //解锁flash保护
	
	for(i = start_sec; i <= end_sec; i++)
	{
		if ((i >= 12) && (i <= 23))
		{
			if ((fmc_sector_erase(CTL_SN(i + 4))) != FMC_READY)
				goto error; //退出后上锁保护
		}
		else if ((i >= 24) && (i <= 27))
		{
			if ((fmc_sector_erase(CTL_SN(i - 12))) != FMC_READY)
				goto error;
		}
		else
		{
			if ((fmc_sector_erase(CTL_SN(i))) != FMC_READY)
				goto error;
		}
			
	}
 
	fmc_lock();        //flash上锁
 
	return FMC_READY;

error:
	fmc_lock(); // flash上锁
	return FMC_WPERR;
}

/*=============================================================================
函数名      : flash_write
功能        : 写入FLASH
输入参数说明: 
				addr: 写入Flash的起始地址, 从0开始的偏移地址
				dwLen : 写入数据的字节数
				buff: 写入数据的buf, 地址必须4字节对齐
返回值说明  : 成功返回0; 失败返回-1或错误码
=============================================================================*/
fmc_state_enum flash_write(uint32_t addr, uint32_t dwLen, uint8_t *buff)
{
		
    fmc_state_enum tRtn;
 
    /* 写入数据的字节数，必须是4字节，即一个page的大小，不足则补齐 */
    if (dwLen % FLASH_PAGE_SIZE)
    {
        dwLen = (dwLen / FLASH_PAGE_SIZE + 1) * FLASH_PAGE_SIZE;
    }
 
    // if (((uint32_t)buff) % 4)
	if(dwLen % 4)
        return FMC_OPERR;
 
    fmc_unlock();
 
    while (dwLen)
    {
		fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR |
                        FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
        tRtn = fmc_word_program(addr, *(uint32_t *)buff);
        if (tRtn != FMC_READY)
        {
			fmc_lock(); //退出后上锁保护
            return tRtn;
        }
 
        addr += FLASH_PAGE_SIZE;
        buff += FLASH_PAGE_SIZE;
        dwLen  -= FLASH_PAGE_SIZE;
    }
 
	fmc_lock();	
 
    return tRtn;
}

//FLASH读取操作：
uint32_t flash_read(uint32_t addr,uint32_t len,uint8_t *buff)
{                           
    uint8_t i;   
    for(i=0; i<len; i++)
    {
        /*直接寻址访问*/
        buff[i] = *(__IO int8_t*)addr;
        addr++;
    }

	return 0;  
}

/*=============================================================================
函数名      : flash_write
功能        : 写入FLASH
输入参数说明: 
				addr: 写入Flash的起始地址, 从0开始的偏移地址
				dwLen : 写入数据的字节数
				buff: 写入数据的buf, 地址必须4字节对齐
返回值说明  : 成功返回0; 失败返回-1或错误码
=============================================================================*/
fmc_state_enum flash_write_byte( uint32_t address, uint8_t len, uint8_t *pdata)
{
    fmc_state_enum 	tRtn;
    uint16_t 		i;    
	uint32_t		addr = address;


    /* 解锁 */
    fmc_unlock();

    /* 操作FMC前先清空STAT 状态寄存器，非常必要*/
    fmc_flag_clear(FMC_FLAG_END | FMC_FLAG_OPERR | FMC_FLAG_WPERR | FMC_FLAG_PGMERR | FMC_FLAG_PGSERR);
    
    for (i=0; i<len; i++)
    {
		// if (addr > 0x0804ba40)
		// printf("addr=[0x%X] value=[0x%X]\r\n", addr, pdata[i]);
        tRtn = fmc_byte_program(addr, pdata[i]);
        if (*((volatile uint8_t*)addr) != pdata[i])
        {
			printf("===ERROR=== addr=[0x%X] value=[0x%X] != [0x%X]\r\n", addr,
				   *((volatile uint8_t *)addr), pdata[i]);
			fmc_lock();	  //退出后上锁保护
			return FMC_OPERR; //校验错误
		}

		if (tRtn != FMC_READY)
        {
			printf("===ERROR=== addr=[0x%X] value=[0x%X] write error!\r\n", addr, pdata[i]);
			fmc_lock(); //退出后上锁保护
			return tRtn;  //写错误
		}
		addr += 1; //地址累加
	}

    /* 上锁 */
    fmc_lock();

    return FMC_READY;
}

