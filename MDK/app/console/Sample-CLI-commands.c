/*
 * FreeRTOS V202112.00
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/******************************************************************************
 *
 * http://www.FreeRTOS.org/cli
 *
 ******************************************************************************/

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* FreeRTOS+CLI includes. */
#include "FreeRTOS_CLI.h"

/* Application includes. */
#include "pin.h"
#include "fmc/fmc.h"
#include "emmc/emmc.h"
#include "fatfs/diskio.h"
#include "fatfs/fatfs_user.h"
#include "rtc/rtc.h"
#include "spi/gd25qxx.h"
#include "upgrade/bootload.h"
#include "pin.h"
#include "comm/basecpld.h"
#include "ymodem/common.h"
#include "hk_ymodem.h"
#include "hk_sensor.h"
#include "ipmb/ipmb.h"
#include "online_check.h"
// #include "sensor_list.h"

#ifndef configINCLUDE_TRACE_RELATED_CLI_COMMANDS
#define configINCLUDE_TRACE_RELATED_CLI_COMMANDS 0
#endif

#ifndef configINCLUDE_QUERY_HEAP_COMMAND
#define configINCLUDE_QUERY_HEAP_COMMAND 1
#endif

// Sensor  list
extern Sensor_t g_sensor_list[16];

/*
 * The function that registers the commands that are defined within this file.
 */
void vRegisterSampleCLICommands(void);

/*
 * Implements the task-stats command.
 */
static BaseType_t prvTaskStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/*
 * Implements the run-time-stats command.
 */
#if (configGENERATE_RUN_TIME_STATS == 1)
static BaseType_t prvRunTimeStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
#endif /* configGENERATE_RUN_TIME_STATS */

/*
 * Implements the echo-three-parameters command.
 */
static BaseType_t prvThreeParameterEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/*
 * Implements the echo-parameters command.
 */
static BaseType_t prvParameterEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);

/*
 * Implements the "query heap" command.
 */
#if (configINCLUDE_QUERY_HEAP_COMMAND == 1)
static BaseType_t prvQueryHeapCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
#endif

/*
 * Implements the "trace start" and "trace stop" commands;
 */
#if (configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1)
static BaseType_t prvStartStopTraceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
#endif

/* Structure that defines the "task-stats" command line command.  This generates
a table that gives information on each task in the system. */
static const CLI_Command_Definition_t xTaskStats =
	{
		"task", /* The command string to type. */
		"\r\ntask:\r\n Displays a table showing the state of each FreeRTOS task\r\n",
		prvTaskStatsCommand, /* The function to run. */
		0					 /* No parameters are expected. */
};

/* Structure that defines the "echo_3_parameters" command line command.  This
takes exactly three parameters that the command simply echos back one at a
time. */
static const CLI_Command_Definition_t xThreeParameterEcho =
	{
		"echo-3-parameters",
		"\r\necho-3-parameters <param1> <param2> <param3>:\r\n Expects three parameters, echos each in turn\r\n",
		prvThreeParameterEchoCommand, /* The function to run. */
		3							  /* Three parameters are expected, which can take any value. */
};

/* Structure that defines the "echo_parameters" command line command.  This
takes a variable number of parameters that the command simply echos back one at
a time. */
static const CLI_Command_Definition_t xParameterEcho =
	{
		"echo-parameters",
		"\r\necho-parameters <...>:\r\n Take variable number of parameters, echos each in turn\r\n",
		prvParameterEchoCommand, /* The function to run. */
		-1						 /* The user can enter any number of commands. */
};

#if (configGENERATE_RUN_TIME_STATS == 1)
/* Structure that defines the "run-time-stats" command line command.   This
generates a table that shows how much run time each task has */
static const CLI_Command_Definition_t xRunTimeStats =
	{
		"run-time-stats", /* The command string to type. */
		"\r\nrun-time-stats:\r\n Displays a table showing how much processing time each FreeRTOS task has used\r\n",
		prvRunTimeStatsCommand, /* The function to run. */
		0						/* No parameters are expected. */
};
#endif /* configGENERATE_RUN_TIME_STATS */

#if (configINCLUDE_QUERY_HEAP_COMMAND == 1)
/* Structure that defines the "query_heap" command line command. */
static const CLI_Command_Definition_t xQueryHeap =
	{
		"heap",
		"\r\nheap:\r\n Displays the free heap space, and minimum ever free heap space.\r\n",
		prvQueryHeapCommand, /* The function to run. */
		0					 /* The user can enter any number of commands. */
};
#endif /* configQUERY_HEAP_COMMAND */

#if configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1
/* Structure that defines the "trace" command line command.  This takes a single
parameter, which can be either "start" or "stop". */
static const CLI_Command_Definition_t xStartStopTrace =
	{
		"trace",
		"\r\ntrace [start | stop]:\r\n Starts or stops a trace recording for viewing in FreeRTOS+Trace\r\n",
		prvStartStopTraceCommand, /* The function to run. */
		1						  /* One parameter is expected.  Valid values are "start" and "stop". */
};
#endif /* configINCLUDE_TRACE_RELATED_CLI_COMMANDS */

static BaseType_t prvFmcTestCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	uint8_t write_data[] = {0x5a, 0xaa, 0xef, 0xbc};
	uint8_t read_data[4] = {0};
	uint8_t ori_data[4] = {0};
	char *buff = (char *)malloc(sizeof(char) * 256);
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (buff == NULL)
	{
		printf("=====prvFmcTestCommand(): malloc failed======\r\n");
		return pdFALSE;
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "*** hkzy fmc R/W/E test ***\r\n");
	sprintf(buff, "Test Addr: 0x%x\r\n", FMC_TEST_ADDRESS);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_read(FMC_TEST_ADDRESS, 4, ori_data);
	sprintf(buff, "Original Read: 0x%x%x%x%x\r\n",
			ori_data[0], ori_data[1], ori_data[2], ori_data[3]);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_write(FMC_TEST_ADDRESS, 4, write_data);
	sprintf(buff, "Write: 0x%x%x%x%x\r\n",
			write_data[0], write_data[1], write_data[2], write_data[3]);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_read(FMC_TEST_ADDRESS, 4, read_data);
	sprintf(buff, "Read: 0x%x%x%x%x\r\n",
			read_data[0], read_data[1], read_data[2], read_data[3]);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_erase_sector(FMC_TEST_ADDRESS, FMC_TEST_ADDRESS + 0x04);
	// strcat( pcWriteBuffer,"Erase: 0x");
	sprintf(buff, "Erase Addr: 0x%x - 0x%x\r\n",
			FMC_TEST_ADDRESS, FMC_TEST_ADDRESS + 0x04);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_read(FMC_TEST_ADDRESS, 4, read_data);
	sprintf(buff, "Read: 0x%x%x%x%x\r\n",
			read_data[0], read_data[1], read_data[2], read_data[3]);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_write(FMC_TEST_ADDRESS, 4, ori_data);
	sprintf(buff, "Original Write: 0x%x%x%x%x\r\n",
			ori_data[0], ori_data[1], ori_data[2], ori_data[3]);
	strncat(pcWriteBuffer, buff, strlen(buff));

	flash_read(FMC_TEST_ADDRESS, 4, read_data);
	sprintf(buff, "Read: 0x%x%x%x%x\r\n",
			read_data[0], read_data[1], read_data[2], read_data[3]);
	strncat(pcWriteBuffer, buff, strlen(buff));
	if (buff)
		free(buff);
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static const CLI_Command_Definition_t xFmcTest =
	{
		"fmc-test", /* The command string to type. */
		"\r\nfmc-test:\r\n FMC read and write test\r\n",
		prvFmcTestCommand, /* The function to run. */
		0				   /* No parameters are expected. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvSdioInfoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	char *buff = (char *)malloc(sizeof(char) * 256);
	uint8_t sd_spec, sd_spec3, sd_spec4, sd_security;
	uint32_t block_count, block_size;
	uint16_t temp_ccc;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (buff == NULL)
	{
		printf("=====prvSdioInfoCommand(): malloc failed======\r\n");
		return pdFALSE;
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "\r\n Card information:");
	sd_spec = (sd_scr[1] & 0x0F000000) >> 24;
	sd_spec3 = (sd_scr[1] & 0x00008000) >> 15;
	sd_spec4 = (sd_scr[1] & 0x00000400) >> 10;
	if (2 == sd_spec)
	{
		if (1 == sd_spec3)
		{
			if (1 == sd_spec4)
			{
				strcat(pcWriteBuffer, "\r\n## Card version 4.xx ##");
			}
			else
			{
				strcat(pcWriteBuffer, "\r\n## Card version 3.0x ##");
			}
		}
		else
		{
			strcat(pcWriteBuffer, "\r\n## Card version 2.00 ##");
		}
	}
	else if (1 == sd_spec)
	{
		strcat(pcWriteBuffer, "\r\n## Card version 1.10 ##");
	}
	else if (0 == sd_spec)
	{
		strcat(pcWriteBuffer, "\r\n## Card version 4.0x ##");
	}

	sd_security = (sd_scr[1] & 0x00700000) >> 20;
	if (2 == sd_security)
	{
		strcat(pcWriteBuffer, "\r\n## SDSC card ##");
	}
	else if (3 == sd_security)
	{
		strcat(pcWriteBuffer, "\r\n## SDHC card ##");
	}
	else if (4 == sd_security)
	{
		strcat(pcWriteBuffer, "\r\n## SDXC card ##");
	}

	block_count = (sd_cardinfo.card_csd.c_size + 1) * 1024;
	block_size = 4096;
	// sprintf( buff,"\r\n## Device size is %dKB ##", sd_card_capacity_get());
	// strcat( pcWriteBuffer, buff);
	sprintf(buff, "\r\n## Block size is %dB ##", block_size);
	strcat(pcWriteBuffer, buff);
	sprintf(buff, "\r\n## Block count is %d ##", block_count);
	strcat(pcWriteBuffer, buff);

	if (sd_cardinfo.card_csd.read_bl_partial)
	{
		strcat(pcWriteBuffer, "\r\n## Partial blocks for read allowed ##");
	}
	if (sd_cardinfo.card_csd.write_bl_partial)
	{
		strcat(pcWriteBuffer, "\r\n## Partial blocks for write allowed ##");
	}
	temp_ccc = sd_cardinfo.card_csd.ccc;
	sprintf(buff, "\r\n## CardCommandClasses is: %x ##", temp_ccc);
	strcat(pcWriteBuffer, buff);
	if ((MMC_CCC_BLOCK_READ & temp_ccc) && (MMC_CCC_BLOCK_WRITE & temp_ccc))
	{
		strcat(pcWriteBuffer, "\r\n## Block operation supported ##");
	}
	if (MMC_CCC_ERASE & temp_ccc)
	{
		strcat(pcWriteBuffer, "\r\n## Erase supported ##");
	}
	if (MMC_CCC_WRITE_PROTECTION & temp_ccc)
	{
		strcat(pcWriteBuffer, "\r\n## Write protection supported ##");
	}
	if (MMC_CCC_LOCK_CARD & temp_ccc)
	{
		strcat(pcWriteBuffer, "\r\n## Lock unlock supported ##");
	}
	if (MMC_CCC_APPLICATION_SPECIFIC & temp_ccc)
	{
		strcat(pcWriteBuffer, "\r\n## Application specific supported ##");
	}
	if (MMC_CCC_IO_MODE & temp_ccc)
	{
		strcat(pcWriteBuffer, "\r\n## I/O mode supported ##");
	}

	if (buff)
		free(buff);
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static const CLI_Command_Definition_t xSdioTest =
	{
		"sdio-info", /* The command string to type. */
		"\r\nsdio-info:\r\n get sdio info\r\n",
		prvSdioInfoCommand, /* The function to run. */
		0					/* No parameters are expected. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvSdTestCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	uint16_t stat = 0;
	// char *fName =  "0:/FATFS.TXT";
	char *fName = UPLOAD_BIOS_FILE;
	char buffer[64] = "HKZY gd32f450 === sdio_fatfs_32g TEST\r\n\0";

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	// rtc_current_time_get(&rtc_initpara);
	stat = fatfs_getfree();
	// stat = fatfs_write(fName,buffer);
	// if (stat == FR_OK)
	//     printf("\r\nFATFS File Write Content:[%s]\r\n",buffer);
	// else
	//     printf("\r\fatfs_write Failed!\r\n");

	stat = fatfs_read(fName, buffer, 64);
	if (stat == FR_OK)
		printf("\r\nFATFS File Read Content:[%s]\r\n", buffer);
	else
		printf("\r\n fatfs_read Failed!\r\n");

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "\r\nEmmc SDIO FAT-FS Test End\r\n");

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static const CLI_Command_Definition_t xSdTest =
	{
		"sd-test", /* The command string to type. */
		"\r\nsd-test:\r\n emmc read write test\r\n",
		prvSdTestCommand, /* The function to run. */
		0				  /* No parameters are expected. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvSdGetSizeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	fatfs_getfree();
	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

static const CLI_Command_Definition_t xSdGetSize =
	{
		"sd-getsize", /* The command string to type. */
		"\r\nsd-getsize:\r\n emmc get size test\r\n",
		prvSdGetSizeCommand, /* The function to run. */
		0					 /* No parameters are expected. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvSpiReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf(pcWriteBuffer, "The parameters were:\r\n");

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		if (pcParameter != NULL)
		{
			uint16_t addr = (uint16_t)strtoul(pcParameter, NULL, 0);
			uint8_t value = 0;

			value = spi_1_read(addr);

			printf("SPI Address [0x%X]=[0x%X]\r\n", addr, value);

			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			strncat(pcWriteBuffer, "SPI test end\r\n", strlen("SPI test end\r\n"));

			/* There might be more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
		else
		{
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t xSpiRead =
	{
		"spi-read", /* The command string to type. */
		"\r\nspi-read:\r\n spi-read [addr]\r\n",
		prvSpiReadCommand, /* The function to run. */
		-1				   /* The user can enter any number of commands. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvSpiWriteCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint16_t input[2] = {0xFF};
	char buff[4] = {0x00};

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		// sprintf( pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Return the parameter string. */
		strncat(buff, pcParameter, (size_t)xParameterStringLength);
		if (input[0] == 0xFF)
			input[0] = (uint16_t)strtoul(buff, NULL, 0);
		else
			input[1] = (uint16_t)strtoul(buff, NULL, 0);

		if (uxParameterNumber == 2U)
		{
			printf("SPI Write Address [0x%X]\r\n", input[0]);
			printf("SPI Write Valud [0x%X]\r\n", input[1]);

			spi_1_write(input[0], input[1]);

			// /* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			strncat(pcWriteBuffer, "SPI test end\r\n", strlen("SPI test end\r\n"));
			input[0] = 0xFF;

			xReturn = pdFALSE;
			uxParameterNumber = 0;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t xSpiWrite =
	{
		"spi-write", /* The command string to type. */
		"\r\nspi-write:\r\n spi-write [addr] [value]\r\n",
		prvSpiWriteCommand, /* The function to run. */
		-1					/* The user can enter any number of commands. */
};

/*!
	\brief      cpu power on
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvPowerOnCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	bc_cpu_power_on();

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, "power on ops end\r\n", strlen("power on ops end\r\n"));

	return pdFALSE;
}

static const CLI_Command_Definition_t xPowerOn =
	{
		"power-on", /* The command string to type. */
		"\r\npower-on:\r\n cpu power on\r\n",
		prvPowerOnCommand, /* The function to run. */
		-1				 /* The user can enter any number of commands. */
};

/*!
	\brief      cpu power off
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvPowerOffCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	bc_cpu_power_off();

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, "power off ops end\r\n", strlen("power off ops end\r\n"));

	return pdFALSE;
}

static const CLI_Command_Definition_t xPowerOff =
	{
		"power-off", /* The command string to type. */
		"\r\npower-off:\r\n cpu power off\r\n",
		prvPowerOffCommand, /* The function to run. */
		-1				 /* The user can enter any number of commands. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvSpiTestCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strncat(pcWriteBuffer, "Merge test end\r\n", strlen("Merge test end\r\n"));

	gpio_spi_select_upgrade();
	vTaskDelay(5);
#define BUFFER_SIZE 256
#define SFLASH_ID 0xC84015
// #define SFLASH_ID_GD 0xC84013
#define SFLASH_ID_GD 0xC86018
#define FLASH_WRITE_ADDRESS 0x000000
#define FLASH_READ_ADDRESS FLASH_WRITE_ADDRESS
	uint8_t tx_buffer[BUFFER_SIZE] = {0};
	uint8_t rx_buffer[BUFFER_SIZE] = {0};
	uint32_t flash_id = 0;
	uint16_t i = 0;
	uint8_t is_successful = 0;

	/* get flash id */
	flash_id = spi_flash_read_id();
	printf("\n\rThe Flash_ID:0x%X\n\r\n\r", flash_id);
	/* flash id is correct */

	if ((SFLASH_ID_GD == flash_id) || (SFLASH_ID == flash_id))
	{

		// printf("\n\rWrite to tx_buffer:\n\r\n\r");
		// /* printf tx_buffer value */
		// for (i = 0; i < BUFFER_SIZE; i++)
		// {
		// 	tx_buffer[i] = i;
		// 	printf("0x%02X ", tx_buffer[i]);
		// 	if (15 == i % 16)
		// 		printf("\n\r");
		// }

		// /* erase the specified flash sector */
		// spi_flash_sector_erase(FLASH_WRITE_ADDRESS);

		// /* write tx_buffer data to the flash */
		// spi_flash_buffer_write(tx_buffer, FLASH_WRITE_ADDRESS, BUFFER_SIZE);

		/* read a block of data from the flash to rx_buffer */
		spi_flash_buffer_read(rx_buffer, FLASH_READ_ADDRESS, BUFFER_SIZE);
		printf("\n\rWrite to rx_buffer:\n\r\n\r");
		/* printf rx_buffer value */
		for (i = 0; i < BUFFER_SIZE; i++)
		{
			printf("0x%02X ", rx_buffer[i]);
			if (15 == i % 16)
				printf("\n\r");
		}

		/* spi qspi flash test passed */
		// if (0 == is_successful)
		{
			printf("\n\rSPI-GD25Q40 Test End!\n\r");
		}
	}
	else
	{
		/* spi flash read id fail */
		printf("\n\rSPI Flash: Read ID Fail!\n\r");
	}

	gpio_spi_select_default();

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "************************\r\n");
	return pdFALSE;
}

static const CLI_Command_Definition_t xSpiTest =
	{
		"spi-test", /* The command string to type. */
		"\r\nspi-test:\r\n spi-test\r\n",
		prvSpiTestCommand, /* The function to run. */
		-1				   /* The user can enter any number of commands. */
};

static BaseType_t prvVersionCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	printf("*** hkzy arm version ***\r\n");
	if (IS_APP_RUN)
		printf("SoftWare Start Addr:[0x%X]\r\n", BOOT_APP_FLASH_START_ADDR);
	else if (IS_APP_RUN_BK)
		printf("SoftWare Start Addr:[0x%X]\r\n", BOOT_APP_FLASH_BK_START_ADDR);
	else
		printf("SoftWare Start Addr:[0x%X]\r\n", BOOT_APP_FLASH_START_ADDR);

	printf("SoftWare Version:[%s]\r\n", SOFTWARE_VERSION);
	printf("HardWare Version:[%s]\r\n", HARDWARE_VERSION);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "************************\r\n");
	return pdFALSE;
}

static const CLI_Command_Definition_t xVersion =
	{
		"version", /* The command string to type. */
		"\r\nversion:\r\n software version & hardware version\r\n",
		prvVersionCommand, /* The function to run. */
		0				   /* No parameters are expected. */
};

static BaseType_t prvGetClkCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	printf("*** arm clk freq ***\r\n");

	uint32_t sysclk = rcu_clock_freq_get(CK_SYS);
	uint32_t ahbclk = rcu_clock_freq_get(CK_AHB);
	uint32_t apb1clk = rcu_clock_freq_get(CK_APB1);
	uint32_t apb2clk = rcu_clock_freq_get(CK_APB2);

	printf("CK_SYS:[%dHz]\r\n", ahbclk);
	printf("CK_AHB:[%dHz]\r\n", ahbclk);
	printf("CK_APB1:[%dHz]\r\n", ahbclk);
	printf("CK_APB2:[%XHz]\r\n", ahbclk);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "************************\r\n");
	return pdFALSE;
}

static const CLI_Command_Definition_t xGetClk =
	{
		"get-clk", /* The command string to type. */
		"\r\nget-clk:\r\n get system clk\r\n",
		prvGetClkCommand, /* The function to run. */
		0				  /* No parameters are expected. */
};

static BaseType_t prvGetPhyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	uint16_t val;
	int duplex, speed;
	ErrStatus ret = ERROR;

	printf("*** phy freq ***\r\n");

	// 读取yt8512速率和双工
	ret = enet_phy_write_read(ENET_PHY_READ, YT8512_ADDRESS, YT8512_REG_SPEC_STATUS, &val);
	if (!ret)
	{
		printf("enet_8512_init read speed and duplex fail\r\n");
		return ret;
	}

	duplex = (val & YT8512_DUPLEX) >> YT8512_DUPLEX_BIT;
	speed = (val & YT8512_SPEED_MODE) >> YT8512_SPEED_MODE_BIT;
	switch (speed)
	{
	case 0:
		printf("yt8512 speed=10 Mbps\r\n");
		break;
	case 1:
		printf("yt8512 speed=100 Mbps\r\n");
		break;
	case 2:
		printf("yt8512 speed=1000 Mbps\r\n");
		break;
	case 3:
		printf("yt8512 speed=reserved\r\n");
		break;
	default:
		break;
	}
	if (duplex == 1)
	{
		printf("yt8512 FULL-duplex\r\n");
	}
	else
	{
		printf("yt8512 HALF-duplex\r\n");
	}

	/* TODO: */
	// 读取yt8512速率和双工
	ret = enet_phy_write_read(ENET_PHY_READ, YT8521_ADDRESS, YT8512_REG_SPEC_STATUS, &val);
	if (!ret)
	{
		printf("enet_8512_init read speed and duplex fail\r\n");
		return ret;
	}

	duplex = (val & YT8512_DUPLEX) >> YT8512_DUPLEX_BIT;
	speed = (val & YT8512_SPEED_MODE) >> YT8512_SPEED_MODE_BIT;
	switch (speed)
	{
	case 0:
		printf("yt8521 speed=10 Mbps\r\n");
		break;
	case 1:
		printf("yt8521 speed=100 Mbps\r\n");
		break;
	case 2:
		printf("yt8521 speed=1000 Mbps\r\n");
		break;
	case 3:
		printf("yt8521 speed=reserved\r\n");
		break;
	default:
		break;
	}
	if (duplex == 1)
	{
		printf("yt8521 FULL-duplex\r\n");
	}
	else
	{
		printf("yt8521 HALF-duplex\r\n");
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer, "************************\r\n");
	return pdFALSE;
}

static const CLI_Command_Definition_t xGetPhy =
	{
		"get-phy", /* The command string to type. */
		"\r\nget-phy:\r\n get phy freq\r\n",
		prvGetPhyCommand, /* The function to run. */
		0				  /* No parameters are expected. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvMdioReadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	char buff[4] = {0x00};
	static uint16_t phy_address = 0U;
	static uint16_t addr = 0U;
	static uint16_t value = 0U;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		// sprintf( pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Return the parameter string. */
		strncat(buff, pcParameter, (size_t)xParameterStringLength);

		if (uxParameterNumber == 1U)
		{
			phy_address = (uint16_t)strtoul(buff, NULL, 0);
			xReturn = pdPASS;
			uxParameterNumber++;
		}
		else if (uxParameterNumber == 2U)
		{
			addr = (uint16_t)strtoul(buff, NULL, 0);

			if (ERROR != enet_phy_write_read(ENET_PHY_READ, phy_address, addr, &value))
				printf("MDIO READ PHY_ADDRESS=[0x%X] [0x%X]=[0x%X] sucess!\r\n", phy_address, addr, value);
			else
				printf("MDIO READ PHY_ADDRESS=[0x%X] [0x%X]=[0x%X] failed!\r\n", phy_address, addr, value);

			// /* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			strcpy(pcWriteBuffer, "************************\r\n");
			xReturn = pdFALSE;
			uxParameterNumber = 0;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t xMdioRead =
	{
		"mr", /* The command string to type. */
		"\r\nmdio-read:\r\n mdio-read [addr]\r\n",
		prvMdioReadCommand, /* The function to run. */
		-1					/* The user can enter any number of commands. */
};

/*!
	\brief      get the card information and print it out by USRAT
	\param[in]  none
	\param[out] none
	\retval     none
*/
static BaseType_t prvMdioWriteCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static uint16_t phy_address = 0U;
	static uint16_t addr = 0U;
	static uint16_t value = 0U;
	char buff[4] = {0xFF};

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		// sprintf( pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Return the parameter string. */
		strncat(buff, pcParameter, (size_t)xParameterStringLength);

		if (uxParameterNumber == 1U)
		{
			phy_address = (uint16_t)strtoul(buff, NULL, 0);
			xReturn = pdPASS;
			uxParameterNumber++;
		}
		else if (uxParameterNumber == 2U)
		{
			addr = (uint16_t)strtoul(buff, NULL, 0);
			xReturn = pdPASS;
			uxParameterNumber++;
		}
		else if (uxParameterNumber == 3U)
		{
			value = (uint16_t)strtoul(buff, NULL, 0);

			if (ERROR != enet_phy_write_read(ENET_PHY_WRITE, phy_address, addr, &value))
				printf("MDIO WRITE PHY_ADDRESS=[0x%X] [0x%X]=[0x%X] sucess!\r\n", phy_address, addr, value);
			else
				printf("MDIO WRITE PHY_ADDRESS=[0x%X] [0x%X]=[0x%X] failed!\r\n", phy_address, addr, value);

			// /* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			strcpy(pcWriteBuffer, "************************\r\n");
			xReturn = pdFALSE;
			uxParameterNumber = 0;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t xMdioWrite =
	{
		"mw", /* The command string to type. */
		"\r\nmdio-write:\r\n mdio-write [addr] [value]\r\n",
		prvMdioWriteCommand, /* The function to run. */
		-1					 /* The user can enter any number of commands. */
};
static BaseType_t prvYmodemDownloadCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	BaseType_t xReturn = pdFALSE;
	/* uart update task */
	ymodem_update_thread();
	return xReturn;
}

static const CLI_Command_Definition_t xYmodemDownload =
	{
		"upgrade", /* The command string to type. */
		"\r\nupgrade:\r\n Ymodem upgrade bios\r\n",
		prvYmodemDownloadCommand, /* The function to run. */
		0						  /* The user can enter any number of commands. */
};

static void sensorGetAlarmLevel()
{
	for (uint8_t i = 0; i < 16; i++)
	{
		g_sensor_list[i].curAlarm = NORMAL;
		if (g_sensor_list[i].unit == 1)
		{
			if (g_sensor_list[i].data.iValue > g_sensor_list[i].slightVal)
				g_sensor_list[i].curAlarm = SLIGHTLY_OVER_UPPER_LIMIT;
			if (g_sensor_list[i].data.iValue > g_sensor_list[i].slightVal)
				g_sensor_list[i].curAlarm = SERIOUSLY_OVER_UPPER_LIMIT;
			if (g_sensor_list[i].data.iValue > g_sensor_list[i].slightVal)
				g_sensor_list[i].curAlarm = DANGER_OVER_UPPER_LIMIT;
		}
		if ((g_sensor_list[i].unit == 2) || (g_sensor_list[i].unit == 3))
		{
			if (g_sensor_list[i].data.fValue > g_sensor_list[i].slightVal)
				g_sensor_list[i].curAlarm = SLIGHTLY_OVER_UPPER_LIMIT;
			if (g_sensor_list[i].data.fValue > g_sensor_list[i].slightVal)
				g_sensor_list[i].curAlarm = SERIOUSLY_OVER_UPPER_LIMIT;
			if (g_sensor_list[i].data.fValue > g_sensor_list[i].slightVal)
				g_sensor_list[i].curAlarm = DANGER_OVER_UPPER_LIMIT;
		}
		if (g_sensor_list[i].unit == 4)
		{
			if (g_sensor_list[i].data.iValue == 0)
				g_sensor_list[i].curAlarm = DANGER_OVER_UPPER_LIMIT;
		}
	}
}

static BaseType_t prvReadSensorCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sensorGetAlarmLevel();

	printf("============================ Sensor List ===========================\r\n");
	printf("|%-2s|%-16s|%-9s|%8s|%8s|%8s|%9s|\r\n",
		   "ID", "Name", "Value", "State", "Slight", "Serious", "Danger");
	for (uint8_t i = 0; i < 16; i++)
	{
		if (g_sensor_list[i].unit == 1)
			printf("|%-2d|%-16s|%7d'C|",
				   i, g_sensor_list[i].name, g_sensor_list[i].data.iValue);

		if ((g_sensor_list[i].unit == 2) || (g_sensor_list[i].unit == 3))
			printf("|%-2d|%-16s|%8.2f%s|",
				   i, g_sensor_list[i].name, g_sensor_list[i].data.fValue,
				   (g_sensor_list[i].unit == 2) ? "V" : "A");

		if (g_sensor_list[i].unit == 4)
			printf("|%-2d|%-16s|%9s|", i, g_sensor_list[i].name,
				   (g_sensor_list[i].data.iValue > 0) ? "OnLine" : "NotOnLine");

		printf("%8s|", (g_sensor_list[i].curAlarm == NORMAL) ? "Normal" : 
				(g_sensor_list[i].curAlarm == SLIGHTLY_OVER_UPPER_LIMIT) ? "Slight":
				(g_sensor_list[i].curAlarm == SERIOUSLY_OVER_UPPER_LIMIT)	? "Serious": 
				(g_sensor_list[i].curAlarm == DANGER_OVER_UPPER_LIMIT)		? "Danger": "NULL");

		if (g_sensor_list[i].unit != 4)
			printf("%8.2f|%8.2f|%9.2f|\r\n",
				   g_sensor_list[i].slightVal, g_sensor_list[i].seriousVal, g_sensor_list[i].dangerVal);
		else
			printf("%8s|%8s|%9s|\r\n", "----", "----", "NotOnLine");
	}

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);
	strcpy(pcWriteBuffer,
		   "====================================================================\r\n");
	return pdFALSE;
}

static const CLI_Command_Definition_t xSysInfo =
	{
		"sensor-info", /* The command string to type. */
		"\r\nsensor-info:\r\n Show All Sensor info\r\n",
		prvReadSensorCommand, /* The function to run. */
		-1					  /* The user can enter any number of commands. */
};

/*-----------------------------------------------------------*/

static BaseType_t prvTaskStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *const pcHeader = "     State   Priority  Stack    #\r\n************************************************\r\n";
	BaseType_t xSpacePadding;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Generate a table of task stats. */
	strcpy(pcWriteBuffer, "Task");
	pcWriteBuffer += strlen(pcWriteBuffer);

	/* Minus three for the null terminator and half the number of characters in
	"Task" so the column lines up with the centre of the heading. */
	configASSERT(configMAX_TASK_NAME_LEN > 3);
	for (xSpacePadding = strlen("Task"); xSpacePadding < (configMAX_TASK_NAME_LEN - 3); xSpacePadding++)
	{
		/* Add a space to align columns after the task's name. */
		*pcWriteBuffer = ' ';
		pcWriteBuffer++;

		/* Ensure always terminated. */
		*pcWriteBuffer = 0x00;
	}
	strcpy(pcWriteBuffer, pcHeader);
	vTaskList(pcWriteBuffer + strlen(pcHeader));

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/
static BaseType_t prvSensorWCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;
	static float input[4] = {0};
	static uint8_t write_buf[4] = {0};

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	memset(pcWriteBuffer, 0x00, xWriteBufferLen);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		// sprintf( pcWriteBuffer, "The three parameters were:\r\n" );

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		// input[uxParameterNumber - 1] = (float)strtoul(pcParameter, NULL, 0);
		input[uxParameterNumber - 1] = (float)strtof(pcParameter, NULL);
		if (uxParameterNumber == 4U)
		{
			memset(write_buf, 0x00, 4);
			write_buf[0] = (uint8_t)input[1];
			write_buf[1] = (uint8_t)input[2];
			write_buf[2] = (uint8_t)input[3];

			if ((0 <= input[0]) && (input[0] <= 9))
			{
				flash_erase_sector(SENSOR_ALARM_VALUE_ADDR + 4 * (uint8_t)input[0],
								   SENSOR_ALARM_VALUE_ADDR + 4 * (uint8_t)input[0] + 4);
				flash_write(SENSOR_ALARM_VALUE_ADDR + (4 * (uint8_t)input[0]), 4, (uint8_t *)write_buf);

				// printf("flash_write addr [0x%X]\r\n",SENSOR_ALARM_VALUE_ADDR + (4 * (uint8_t)input[0]));
				g_sensor_list[(uint8_t)input[0]].slightVal = input[1];
				g_sensor_list[(uint8_t)input[0]].seriousVal = input[2];
				g_sensor_list[(uint8_t)input[0]].dangerVal = input[3];
			}
			else
			{
				printf("ERROR:unknown sensor");
			}

			xReturn = pdFALSE;
			uxParameterNumber = 0;
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			strncat(pcWriteBuffer, "  sensor warn_value write end\r\n **********************\r\n",
					strlen("  sensor warn_value write end\r\n **********************\r\n"));
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;
}

static const CLI_Command_Definition_t xAlarmSet =
	{
		"alarm-set", /* The command string to type. */
		"\r\nalarm-set\r\n alarm-set [sensorID] [slightVal] [seriousVal] [dangerVal]",
		prvSensorWCommand, /* The function to run. */
		4				   /* The user can enter any number of commands. */
};

#if (configINCLUDE_QUERY_HEAP_COMMAND == 1)

static BaseType_t prvQueryHeapCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sprintf(pcWriteBuffer, "Current free heap %d bytes, minimum ever free heap %d bytes\r\n", (int)xPortGetFreeHeapSize(), (int)xPortGetMinimumEverFreeHeapSize());

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

#endif /* configINCLUDE_QUERY_HEAP */
/*-----------------------------------------------------------*/

#if (configGENERATE_RUN_TIME_STATS == 1)

static BaseType_t prvRunTimeStatsCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *const pcHeader = "  Abs Time      % Time\r\n****************************************\r\n";
	BaseType_t xSpacePadding;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Generate a table of task stats. */
	strcpy(pcWriteBuffer, "Task");
	pcWriteBuffer += strlen(pcWriteBuffer);

	/* Pad the string "task" with however many bytes necessary to make it the
	length of a task name.  Minus three for the null terminator and half the
	number of characters in	"Task" so the column lines up with the centre of
	the heading. */
	for (xSpacePadding = strlen("Task"); xSpacePadding < (configMAX_TASK_NAME_LEN - 3); xSpacePadding++)
	{
		/* Add a space to align columns after the task's name. */
		*pcWriteBuffer = ' ';
		pcWriteBuffer++;

		/* Ensure always terminated. */
		*pcWriteBuffer = 0x00;
	}

	strcpy(pcWriteBuffer, pcHeader);
	vTaskGetRunTimeStats(pcWriteBuffer + strlen(pcHeader));

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

#endif /* configGENERATE_RUN_TIME_STATS */
/*-----------------------------------------------------------*/

static BaseType_t prvThreeParameterEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf(pcWriteBuffer, "The three parameters were:\r\n");

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		/* Sanity check something was returned. */
		configASSERT(pcParameter);

		/* Return the parameter string. */
		memset(pcWriteBuffer, 0x00, xWriteBufferLen);
		sprintf(pcWriteBuffer, "%d: ", (int)uxParameterNumber);
		strncat(pcWriteBuffer, pcParameter, (size_t)xParameterStringLength);
		strncat(pcWriteBuffer, "\r\n", strlen("\r\n"));

		/* If this is the last of the three parameters then there are no more
		strings to return after this one. */
		if (uxParameterNumber == 3U)
		{
			/* If this is the last of the three parameters then there are no more
			strings to return after this one. */
			xReturn = pdFALSE;
			uxParameterNumber = 0;
		}
		else
		{
			/* There are more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

static BaseType_t prvParameterEchoCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t xParameterStringLength, xReturn;
	static UBaseType_t uxParameterNumber = 0;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	if (uxParameterNumber == 0)
	{
		/* The first time the function is called after the command has been
		entered just a header string is returned. */
		sprintf(pcWriteBuffer, "The parameters were:\r\n");

		/* Next time the function is called the first parameter will be echoed
		back. */
		uxParameterNumber = 1U;

		/* There is more data to be returned as no parameters have been echoed
		back yet. */
		xReturn = pdPASS;
	}
	else
	{
		/* Obtain the parameter string. */
		pcParameter = FreeRTOS_CLIGetParameter(
			pcCommandString,		/* The command string itself. */
			uxParameterNumber,		/* Return the next parameter. */
			&xParameterStringLength /* Store the parameter string length. */
		);

		if (pcParameter != NULL)
		{
			/* Return the parameter string. */
			memset(pcWriteBuffer, 0x00, xWriteBufferLen);
			sprintf(pcWriteBuffer, "%d: ", (int)uxParameterNumber);
			strncat(pcWriteBuffer, (char *)pcParameter, (size_t)xParameterStringLength);
			strncat(pcWriteBuffer, "\r\n", strlen("\r\n"));

			/* There might be more parameters to return after this one. */
			xReturn = pdTRUE;
			uxParameterNumber++;
		}
		else
		{
			/* No more parameters were found.  Make sure the write buffer does
			not contain a valid string. */
			pcWriteBuffer[0] = 0x00;

			/* No more data to return. */
			xReturn = pdFALSE;

			/* Start over the next time this command is executed. */
			uxParameterNumber = 0;
		}
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

#if configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1

static BaseType_t prvStartStopTraceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString)
{
	const char *pcParameter;
	BaseType_t lParameterStringLength;

	/* Remove compile time warnings about unused parameters, and check the
	write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	write buffer length is adequate, so does not check for buffer overflows. */
	(void)pcCommandString;
	(void)xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the parameter string. */
	pcParameter = FreeRTOS_CLIGetParameter(
		pcCommandString,		/* The command string itself. */
		1,						/* Return the first parameter. */
		&lParameterStringLength /* Store the parameter string length. */
	);

	/* Sanity check something was returned. */
	configASSERT(pcParameter);

	/* There are only two valid parameter values. */
	if (strncmp(pcParameter, "start", strlen("start")) == 0)
	{
		/* Start or restart the trace. */
		vTraceStop();
		vTraceClear();
		vTraceStart();

		sprintf(pcWriteBuffer, "Trace recording (re)started.\r\n");
	}
	else if (strncmp(pcParameter, "stop", strlen("stop")) == 0)
	{
		/* End the trace, if one is running. */
		vTraceStop();
		(pcWriteBuffer, "Stopping trace recording.\r\n");
	}
	else
	{
		sprintf(pcWriteBuffer, "Valid parameters are 'start' and 'stop'.\r\n");
	}

	/* There is no more data to return after this single string, so return
	pdFALSE. */
	return pdFALSE;
}

#endif /* configINCLUDE_TRACE_RELATED_CLI_COMMANDS */

void vRegisterSampleCLICommands(void)
{
	/* Register all the command line commands defined immediately above. */
	// FreeRTOS_CLIRegisterCommand(&xTaskStats);
	// FreeRTOS_CLIRegisterCommand(&xThreeParameterEcho);
	// FreeRTOS_CLIRegisterCommand(&xParameterEcho);
	// FreeRTOS_CLIRegisterCommand(&xFmcTest);
	// FreeRTOS_CLIRegisterCommand(&xSdioTest);
	// FreeRTOS_CLIRegisterCommand(&xSdTest);
	// FreeRTOS_CLIRegisterCommand(&xSdGetSize); // test
	// FreeRTOS_CLIRegisterCommand(&xSpiRead);
	// FreeRTOS_CLIRegisterCommand(&xSpiWrite);

	// FreeRTOS_CLIRegisterCommand(&xMdioRead);
	// FreeRTOS_CLIRegisterCommand(&xMdioWrite);

	FreeRTOS_CLIRegisterCommand(&xPowerOn);
	FreeRTOS_CLIRegisterCommand(&xPowerOff);
	FreeRTOS_CLIRegisterCommand(&xVersion);
	// FreeRTOS_CLIRegisterCommand(&xGetClk);
	// FreeRTOS_CLIRegisterCommand(&xGetPhy);
	// FreeRTOS_CLIRegisterCommand(&xSpiTest);
	FreeRTOS_CLIRegisterCommand(&xSysInfo);
	/* upgrade bios & bmc */
	FreeRTOS_CLIRegisterCommand(&xYmodemDownload);

	/* sensor warn_value read */
	// FreeRTOS_CLIRegisterCommand(&xSensorWarnR);
	/* sensor warn_value write */
	FreeRTOS_CLIRegisterCommand(&xAlarmSet);

	// #if (configGENERATE_RUN_TIME_STATS == 1)
	// 	{
	// 		FreeRTOS_CLIRegisterCommand(&xRunTimeStats);
	// 	}
	// #endif

	// #if (configINCLUDE_QUERY_HEAP_COMMAND == 1)
	// 	{
	// 		FreeRTOS_CLIRegisterCommand(&xQueryHeap);
	// 	}
	// #endif

	// #if (configINCLUDE_TRACE_RELATED_CLI_COMMANDS == 1)
	// 	{
	// 		FreeRTOS_CLIRegisterCommand(&xStartStopTrace);
	// 	}
	// #endif
}
