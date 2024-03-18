#ifndef __REFEREETASK_H
#define __REFEREETASK_H
#include "main.h"

#include "fifo.h"
#include "RefereeBehaviour.h"
#define USART_RX_BUF_LENGHT     144
#define REFEREE_FIFO_BUF_LENGTH 1024
	
void RefereeThread(void * argument);	
extern	uint8_t usart5_buf1[USART_RX_BUF_LENGHT];
extern	uint8_t usart5_buf2[USART_RX_BUF_LENGHT];
extern uint8_t usart5_flag;
extern fifo_s_t referee_fifo;
extern uint8_t referee_fifo_buf[REFEREE_FIFO_BUF_LENGTH];
extern unpack_data_t referee_unpack_obj;

void referee_unpack_fifo_data(void);

#endif