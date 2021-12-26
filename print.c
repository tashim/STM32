/*
 * print.c
 *
 *  Created on: Apr 28, 2021
 *      Author: e_tas
 */
/* Private user code ---------------------------------------------------------*/
#include "cmsis_os.h"
#include <string.h>

#include <stdio.h>
#include <stdlib.h>
#include "eth_rtg.h"


/* USER CODE BEGIN 0 */
//for printf
extern UART_HandleTypeDef huart1;

int __io_putchar(int ch)
{
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART2 and Loop until the end of transmission */
	char udp_mes[2]={0};
	// UART1 send char
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
	udp_mes[0]=ch;
	// UDP send char
	RTG_Udp_Send(udp_mes);
	return ch;
}

// for scanf
int _read(int file, char *ptr, int len)
{
	int ch=0;
	HAL_UART_Receive(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,HAL_MAX_DELAY);
	if(ch==13)
	{
		ch=10;
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,HAL_MAX_DELAY);
	}
	else if(ch==8)
	{
		ch=0x30;
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,HAL_MAX_DELAY);
	}

	*ptr=ch;

	return 1;
}
