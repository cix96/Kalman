/* Inicializacija USART2 */

#ifndef USART_H
#define USART_H

#include <stm32f4xx.h> 						// common stuff
#include <stm32f4xx_gpio.h> 			// gpio control
#include <stm32f4xx_rcc.h> 				// reset anc clocking
#include <stm32f4xx_usart.h>			// serijska komunikacija
#include <stm32f4_discovery.h>
#include <stdio.h>

#define  	BUFSIZE 	16
#define 	BAUDRATE 	115200

void USART2_Init(void);						// inicializacija USART2 periferije
void USART2_SendChar(char c);			// slanje znakova sve dok je pun buffer
int USART2_Dequeue(char *c);			// pop character from receive FIFO
void send_message(char *s);				// slanje poruke

#endif
