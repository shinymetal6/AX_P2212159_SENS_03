/*
 * uart.c
 *
 *  Created on: Jan 10, 2023
 *      Author: fil
 */

#include "main.h"

enum 	UART_STATE		uart_state;

void tx_uart(void)
{
uint8_t	i,k,len;
	HAL_GPIO_WritePin(USART1_RE_GPIO_Port, USART1_RE_Pin, GPIO_PIN_SET);
	len = strlen((char *)SystemVar.uart_buf);
	bzero(SystemVar.uart_txbuf,sizeof(SystemVar.uart_txbuf));
	for(i=0,k=2;i<len;i++,k++)
		SystemVar.uart_txbuf[k] = SystemVar.uart_buf[i];
	HAL_UART_Transmit_IT(&huart1, SystemVar.uart_txbuf, len+4);
}

void uart_rxpacket_analyze(void)
{
	/* <Gn>0x0d */
	switch(uart_state)
	{
	case	WAIT_FOR_START:
		if ( SystemVar.uart_rxchar == '<' )
		{
			SystemVar.uart_rxbuf[0] = SystemVar.uart_rxchar;
			SystemVar.uart_buf_index = 1;
			uart_state = WAIT_FOR_END;
		}
		break;
	case	WAIT_FOR_END:
		if (( SystemVar.uart_buf_index == 2) && ( SystemVar.uart_rxchar != ram_board_parameters.address))
		{
			SystemVar.uart_buf_index = 0;
			uart_state = WAIT_FOR_START;
		}
		if ( SystemVar.uart_rxchar == '>' )
		{
			SystemVar.uart_rxbuf[SystemVar.uart_buf_index] = SystemVar.uart_rxchar;
			SystemVar.uart_buf_rxed = SystemVar.uart_buf_index;
			SystemVar.uart_buf_index = 0;
			uart_state = WAIT_FOR_START;
			SystemVar.system_flags |= FLAGS_PKT_VALID;
		}
		else
		{
			SystemVar.uart_rxbuf[SystemVar.uart_buf_index] = SystemVar.uart_rxchar;
			SystemVar.uart_buf_index++;
			if ( SystemVar.uart_buf_index > (UART_RXBUF_LEN-2) )
			{
				SystemVar.uart_buf_index = 0;
				uart_state = WAIT_FOR_START;
			}
		}
		break;
	}
}
