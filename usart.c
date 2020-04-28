/* USART.c */

#include <usart.h>



// RX FIFO buffer
char RX_BUFFER[BUFSIZE];
int RX_BUFFER_HEAD, RX_BUFFER_TAIL;

// TX state flag
uint8_t TxReady;


// init USART1
void USART2_Init(void){

	GPIO_InitTypeDef 			GPIO_InitStruct;
	USART_InitTypeDef 		USART_InitStruct;
	NVIC_InitTypeDef 			NVIC_InitStructure;

	// enable peripheral clocks (note: different bus interfaces for each peripheral!)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	// map port C pins for alternate function
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;																						// Pins 5 (TX) will be used for USART2
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 																				// GPIO pins defined as alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; 																		// I/O pins speed (signal rise time)
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; 																			// push-pull output
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; 																				// activates pullup resistors
	GPIO_Init(GPIOD, &GPIO_InitStruct); 																							// set chosen pins

	// set alternate function to USART2 (from multiple possible alternate function choices)
	// pins will automatically be assigned to TX - refer to datasheet to see AF mappings
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource5, GPIO_AF_USART2); 
	
	// use USART_InitStruct to config USART2 peripheral
	USART_InitStruct.USART_BaudRate = BAUDRATE; 																			// set baudrate from define
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;													// 8 data bits
	USART_InitStruct.USART_StopBits = USART_StopBits_1; 															// 1 stop bit
	USART_InitStruct.USART_Parity = USART_Parity_No; 																	// no parity check
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 			// no HW control flow
	USART_InitStruct.USART_Mode = USART_Mode_Tx; 																			// enable both character transmit 
	USART_Init(USART2, &USART_InitStruct);																						// set USART1 peripheral

	// set interrupt triggers for USART2 ISR (but do not enable USART2 interrupts yet)
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);																		// should be disbled
	USART_ITConfig(USART2, USART_IT_TC, ENABLE); 																			// transmission completed event (for reseting TxReady flag)
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); 																		// character received (to trigger buffering of new character)
	TxReady = 1; 																																			// USART2 is ready to transmit
	RX_BUFFER_HEAD = 0; RX_BUFFER_TAIL = 0; 																					// clear rx buffer

	// prepare NVIC to receive USART1 IRQs
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; 																// configure USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;													// max. priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 																// max. priority
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 																	// enable USART1 interrupt in NVIC
	NVIC_Init(&NVIC_InitStructure); 																									// set NVIC for USART1 IRQ
	
	// enables USART1 interrupt generation
	USART_Cmd(USART2, ENABLE);

}

void USART2_IRQHandler(void){

	static char rx_char;
	static char rx_head;

	// RX event
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET){
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
			rx_char = USART_ReceiveData(USART2);
			// check for buffer overrun:
			rx_head = RX_BUFFER_HEAD + 1;
			if (rx_head == BUFSIZE) rx_head = 0;
			if (rx_head != RX_BUFFER_TAIL){
			// adding new char will not cause buffer overrun:
			RX_BUFFER[RX_BUFFER_HEAD] = rx_char;
			RX_BUFFER_HEAD = rx_head; // update head
			}
	}
	
	// TX event
	if (USART_GetITStatus(USART2, USART_IT_TC) == SET){
			USART_ClearITPendingBit(USART2, USART_IT_TC);
			TxReady = 1;
	}
}

void USART2_SendChar(char c){
		
		while(!TxReady);
		USART_SendData(USART2, c);
		TxReady = 0;
}


int USART2_Dequeue(char* c){
	
		int ret;
		ret = 0;
		*c = 0;
		NVIC_DisableIRQ(USART2_IRQn);
		if (RX_BUFFER_HEAD != RX_BUFFER_TAIL){
				*c = RX_BUFFER[RX_BUFFER_TAIL];
				RX_BUFFER_TAIL++;
				if (RX_BUFFER_TAIL == BUFSIZE) RX_BUFFER_TAIL = 0;
						ret = 1;
		}
		NVIC_EnableIRQ(USART2_IRQn);

		return ret;
}

void send_message(char *s){
	
			while(*s!='\0'){
				while(!TxReady);
				USART2_SendChar(*s);
				s++;
			}	
}
