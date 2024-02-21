/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
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

/*
	BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER FOR UART0.
*/

#include "stdint.h"
#include "stdio.h"

#include "gd32f4xx.h"
#include "gd32f450z_eval.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#include "pin.h"
#include "console/serial.h"

/*-----------------------------------------------------------*/

/* Misc defines. */
#define serINVALID_QUEUE ((QueueHandle_t)0)
#define serNO_BLOCK ((TickType_t)0)
#define serTX_BLOCK_TIME (40 / portTICK_PERIOD_MS)
#define UART4_DATA_ADDRESS ((uint32_t)0x40011004)

#define ARRAYNUM(arr_name) (uint32_t)(sizeof(arr_name) / sizeof(*(arr_name)))
/*-----------------------------------------------------------*/

/* The queue used to hold received characters. */
static QueueHandle_t xRxedChars;
static QueueHandle_t xCharsForTx;

/*-----------------------------------------------------------*/

/* UART interrupt handler. */
void vUARTInterruptHandler(void);

/*-----------------------------------------------------------*/

void usart_dma_config(void)
{
	//	dma_single_data_parameter_struct dma_init_struct;  //dam未使用删除
	//	/* enable DMA0 */
	//	rcu_periph_clock_enable(RCU_DMA0);
	//	/* deinitialize DMA channel7(UART4 rx) */
	//	dma_deinit(DMA0, DMA_CH2);
	//	dma_init_struct.direction = DMA_PERIPH_TO_MEMORY;
	//	dma_init_struct.memory0_addr = (uint32_t)rx_buffer;
	//	dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
	//	dma_init_struct.periph_memory_width = DMA_MEMORY_WIDTH_8BIT;
	//	dma_init_struct.number = ARRAYNUM(rx_buffer);
	//	dma_init_struct.periph_addr = UART4_DATA_ADDRESS;
	//	dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
	//	dma_init_struct.priority = DMA_PRIORITY_MEDIUM;
	//	dma_single_data_mode_init(DMA0, DMA_CH2, &dma_init_struct);
	//	/* configure DMA mode */
	//	dma_circulation_disable(DMA0, DMA_CH2);
	//	dma_channel_subperipheral_select(DMA0, DMA_CH2, DMA_SUBPERI0);

	// dma_deinit(DMA0, DMA_CH7);
	// dma_init_struct.direction = DMA_MEMORY_TO_PERIPH;
	// dma_init_struct.memory0_addr = (uint32_t)tx_buffer;
	// dma_single_data_mode_init(DMA0, DMA_CH7, &dma_init_struct);
	/* configure DMA mode */
	// dma_circulation_disable(DMA0, DMA_CH7);
	// dma_channel_subperipheral_select(DMA0, DMA_CH7, DMA_SUBPERI4);
	return;
}

/*
 * See the serial2.h header file.
 */
xComPortHandle xSerialPortInitMinimal(unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength)
{
	xComPortHandle xReturn;

	/* Create the queues used to hold Rx/Tx characters. */
	xRxedChars = xQueueCreate(uxQueueLength, (unsigned portBASE_TYPE)sizeof(signed char));
	xCharsForTx = xQueueCreate(uxQueueLength + 1, (unsigned portBASE_TYPE)sizeof(signed char));

	/* If the queue/semaphore was created correctly then setup the serial port
	hardware. */
	if ((xRxedChars != serINVALID_QUEUE) && (xCharsForTx != serINVALID_QUEUE))
	{
		/* USART interrupt configuration */
		nvic_irq_enable(UART4_IRQn, 2, 2);

		/* configure UART4 */
		rcu_periph_clock_enable(RCU_GPIOC);
		rcu_periph_clock_enable(RCU_GPIOD);
		/* enable USART clock */
		rcu_periph_clock_enable(RCU_UART4);
		/* connect port to USARTx_Tx */
		gpio_af_set(BMC_UART4_TX_PORT, GPIO_AF_8, BMC_UART4_TX_PIN);
		/* connect port to USARTx_Rx */
		gpio_af_set(BMC_UART4_RX_PORT, GPIO_AF_8, BMC_UART4_RX_PIN);
		/* configure USART Tx as alternate function push-pull */
		gpio_mode_set(BMC_UART4_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BMC_UART4_TX_PIN);
		gpio_output_options_set(BMC_UART4_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BMC_UART4_TX_PIN);
		/* configure USART Rx as alternate function push-pull */
		gpio_mode_set(BMC_UART4_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BMC_UART4_RX_PIN);
		gpio_output_options_set(BMC_UART4_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BMC_UART4_RX_PIN);
		/* USART configure */
		usart_deinit(UART4);
		usart_baudrate_set(UART4, ulWantedBaud);
		usart_word_length_set(UART4, USART_WL_8BIT);
		usart_stop_bit_set(UART4, USART_STB_1BIT);
		usart_parity_config(UART4, USART_PM_NONE);
		usart_hardware_flow_rts_config(UART4, USART_RTS_DISABLE);
		usart_hardware_flow_cts_config(UART4, USART_CTS_DISABLE);
		usart_receive_config(UART4, USART_RECEIVE_ENABLE);
		usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
		usart_interrupt_enable(UART4, USART_INT_IDLE);
		usart_enable(UART4);
		// /* enable UART4 receive interrupt */
		usart_interrupt_enable(UART4, USART_INT_RBNE);
		// /* enable UART4 transmit interrupt */
		// usart_interrupt_enable(UART4, USART_INT_TBE);
		/* enable UART4 IDLE interrupt */

		// /* configure USART DMA */
		// usart_dma_config();
		// /* enable UART4 DMA channel transmission and reception */
		// // dma_channel_enable(DMA0, DMA_CH7);
		// dma_channel_enable(DMA0, DMA_CH2);

		// /* configure USART DMA */  //dam未使用删除
		// usart_dma_config();
		// /* enable UART4 DMA channel transmission and reception */
		// // dma_channel_enable(DMA0, DMA_CH7);
		// dma_channel_enable(DMA0, DMA_CH2);

		// /* USART DMA enable for transmission and reception */
		// usart_dma_transmit_config(UART4, USART_DENT_ENABLE);
		// usart_dma_receive_config(UART4, USART_DENR_ENABLE);
	}
	else
	{
		xReturn = (xComPortHandle)0;
	}

	/* This demo file only supports a single port but we have to return
	something to comply with the standard demo header file. */
	return xReturn;
}
/*-----------------------------------------------------------*/
signed portBASE_TYPE xSerialGetChar(xComPortHandle pxPort, signed char *pcRxedChar, TickType_t xBlockTime)
{
	/* The port handle is not required as this driver only supports one port. */
	(void)pxPort;
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if (xQueueReceive(xRxedChars, pcRxedChar, xBlockTime))
	{
		return pdPASS;
	}
	else
	{
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

void vSerialPutString(xComPortHandle pxPort, const signed char *const pcString, unsigned short usStringLength)
{
	signed char *pxNext;

	/* A couple of parameters that this port does not use. */
	(void)usStringLength;
	(void)pxPort;

	/* NOTE: This implementation does not handle the queue being full as no
	block time is used! */

	/* The port handle is not required as this driver only supports UART1. */
	(void)pxPort;

	/* Send each character in the string, one at a time. */
	pxNext = (signed char *)pcString;
	while (*pxNext)
	{
		xSerialPutChar(pxPort, *pxNext, serNO_BLOCK);
		pxNext++;
	}
}
/*-----------------------------------------------------------*/

signed portBASE_TYPE xSerialPutChar(xComPortHandle pxPort, signed char cOutChar, TickType_t xBlockTime)
{
	signed portBASE_TYPE xReturn;
	uint8_t cChar;

	if (xQueueSend(xCharsForTx, &cOutChar, xBlockTime) == pdPASS)
	{
		xReturn = pdPASS;
		if (xQueueReceive(xCharsForTx, &cChar, 0) == pdTRUE)
		{
			usart_data_transmit(UART4, (uint8_t)cChar);
			while (RESET == usart_flag_get(UART4, USART_FLAG_TBE))
				;
		}
	}
	else
	{
		xReturn = pdFAIL;
	}

	return xReturn;
}
/*-----------------------------------------------------------*/

void vSerialClose(xComPortHandle xPort)
{
	/* Not supported as not required by the demo application. */
}
/*-----------------------------------------------------------*/

/*!
	\brief      this function handles USART RBNE interrupt request and TBE interrupt request
	\param[in]  none
	\param[out] none
	\retval     none
*/
void UART4_IRQHandler(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	uint8_t rx_data = 0;
	// uint16_t rx_len = 0;
	// int i;
	// if ((RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_IDLE)) &&
	// 	(RESET == dma_flag_get(DMA1, DMA_CH2, DMA_INTF_FTFIF))) //空闲中断

	if ((RESET != usart_interrupt_flag_get(UART4, USART_INT_FLAG_IDLE))) //空闲中断
	{
		rx_data = (uint8_t)usart_data_receive(UART4);

		usart_interrupt_flag_clear(UART4, USART_INT_FLAG_RBNE); // 清除空闲中断标志位
		// rx_len = sizeof(rx_buffer) - dma_transfer_number_get(DMA0, DMA_CH2);
		xQueueSendFromISR(xRxedChars, &rx_data, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

		// for(i = 0; i<rx_len;i++)    //删除DMA
		// {
		//   	xQueueSendFromISR(xRxedChars, &rx_data, &xHigherPriorityTaskWoken);
		//   	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
		// }
		// dma_channel_disable(DMA0, DMA_CH2);                     // 关闭DMA传输
		// dma_interrupt_flag_clear(DMA0,DMA_CH2,DMA_INT_FLAG_FTF);
		// dma_channel_enable(DMA0, DMA_CH2);
	}
}
/*-----------------------------------------------------------*/

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
	usart_data_transmit(UART4, (uint8_t)ch);
	while (RESET == usart_flag_get(UART4, USART_FLAG_TBE))
		;
	return ch;
}
