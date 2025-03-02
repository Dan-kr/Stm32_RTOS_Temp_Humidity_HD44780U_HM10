#include "uart2_write.h"
#include "cmsis_os.h"
#include "main.h"

int uart2_write(int ch)
	{
	/*Make sure the transmit data register is empty*/
	while(!(USART2->SR & USART_SR_TXE)){}

	 /*Write to transmit data register*/
	USART2->DR	=  (ch & 0xFF);
	return ch;
	}

int __io_putchar(int ch)
	{
	uart2_write(ch);
	return ch;
	}