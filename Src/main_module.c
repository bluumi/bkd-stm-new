/*
 * main_module.c
 *
 *  Created on: May 13, 2020
 *      Author: ipumpurs
 */

/* Includes */
#include "main_module.h"
#include "main.h"

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

/* External Typedef */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;


#ifndef LF
	#define LF '\x0A'
#endif


/*-------------- Receive buffer definitions -----------------*/




/*
 *
 *
 *
 * For any index we do operation and then increase (modulo 12)
 *
          0   1   2   3   4   5   6   7   8   9   10  11
buffer:  [h] [e] [l] [l] [o] [ ] [w] [o] [r] [l] [d] [ ]  (12 bytes)
read_i:                           ^
write_i:                                              ^

Special cases:

1. FULL
buffer:  [X] [X] [X] [X] [X] [ ] [w] [o] [r] [l] [d] [X]  (12 bytes)
read_i:                           ^
write_i:                      ^

2. EMPTY
buffer:  [ ] [ ] [ ] [ ] [ ] [ ] [ ] [ ] [ ] [ ] [ ] [ ]  (12 bytes)
read_i:                       ^
write_i:                      ^
*/
#define RECEIVE_BUFFER_LEN	256
volatile uint8_t receive_buffer[RECEIVE_BUFFER_LEN];		//!< circular receive buffer
volatile size_t rec_buf_read_i = 0;
volatile size_t rec_buf_write_i = 0;

uint8_t receive_buffer_peek_data[256];		//!< a copy of receive buffer; not circular

void receive_buffer_put_byte(uint8_t c)
{
	size_t new_write_i = (rec_buf_write_i + 1) % RECEIVE_BUFFER_LEN;
	if (new_write_i == rec_buf_read_i)
		return;

	receive_buffer[rec_buf_write_i] = c;
	rec_buf_write_i = new_write_i;
}

size_t receive_buffer_peek(void)
{
	size_t read_i = rec_buf_read_i;
	size_t write_i = rec_buf_write_i;
	size_t len = 0;

	for (;; ++len)
	{
		// checking if byte present in receive buffer:
		if (read_i == write_i)
			break;
		receive_buffer_peek_data[len] = receive_buffer[read_i];
		read_i = (read_i + 1) % RECEIVE_BUFFER_LEN;
	}
	return len;
}

bool receive_buffer_erase(size_t len)
{
	for (size_t i = 0; i < len; ++i)
	{
		// checking if can erase:
		if (rec_buf_read_i == rec_buf_write_i)
			return false;
		rec_buf_read_i = (rec_buf_read_i + 1) % RECEIVE_BUFFER_LEN;
	}
	return true;
}

/*------------------------------------------------------*/

void main_module(void)
{
	char msg[64];

	HAL_UART_Receive_IT(&huart2, (uint8_t*)receive_buffer_put_byte, 0); // infinite receive

	while(1)
	{

		//HAL_UART_StateTypeDef state =  HAL_UART_GetState(&huart2);
		HAL_Delay(10);

		// checking so-far received data using peek:
		size_t len = receive_buffer_peek();

		if(len != 0)
		{
			bool LF_found = false;
			size_t LF_position;
			for (LF_position = 0; LF_position < len; ++LF_position)
			{
				if (receive_buffer_peek_data[LF_position] == LF)
				{
					LF_found = true;
					break;
				}
			}

			if (LF_found)
			{
				// processing command - bytes from 0...(LF_position-1), length = LF_position

				//! TODO
				//if (isCommand("PRINT"))

				if ((LF_position == 5) && (strncmp((char*)receive_buffer_peek_data, "PRINT", 5) == 0))
				{
					sprintf(msg,"Print..\r\n");
					HAL_UART_Transmit_IT(&huart1, (uint8_t*)msg, strlen(msg));
					while(HAL_UART_GetState(&huart1) != HAL_UART_STATE_READY);											//!< TODO
					HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
				} else if ((LF_position == 2) && (strncmp((char*)receive_buffer_peek_data, "ON", 2) == 0))
				{
					HAL_GPIO_WritePin(GPIOB, LD2_Pin, GPIO_PIN_SET);
				}
				// erasing this string + LF symbol
				receive_buffer_erase(LF_position+1);
			}
		}
	}
}