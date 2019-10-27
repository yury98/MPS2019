#ifndef __UART_H__
#define __UART_H__

void UART1Config(void);
void uart_Rcv(void);
void uart_Send(char* buf, unsigned int buf_size);
#endif
