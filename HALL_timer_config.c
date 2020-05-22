/* INICIJALIZACIJA TIMERA ZA HALL SENZORE */

#include <HALL_timer_config.h>

void timer4_init(void){

		GPIO_InitTypeDef 						GPIO_InitStruct;
		TIM_TimeBaseInitTypeDef 		TIM_TimeBaseStructure;
		TIM_ICInitTypeDef 					TIM_ICInitStructure;
		NVIC_InitTypeDef 						NVIC_InitStructure;
	
		// enable pin clock, GPIOD pin 6,7,8 za timer4
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 
		
		GPIO_InitStruct.GPIO_Pin 		= GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 ;			// HALL1 / HALL2 / HALL3
		GPIO_InitStruct.GPIO_Mode 	= GPIO_Mode_AF;																// ALTERNATE FUNCTION FOR TIIMER
		GPIO_InitStruct.GPIO_Speed 	= GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType 	= GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd 	= GPIO_PuPd_DOWN;	
		GPIO_Init(GPIOD, &GPIO_InitStruct);																				// postavljanje odabranih pinova

		GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); 									// Connect TIM4 pin (PB6) to AF2 
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 									// Connect TIM4 pin (PB7) to AF2 
		GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); 									// Connect TIM4 pin (PB8) to AF2 
	
		// enable timer clock, TIMER4 
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
		// inicijalizacija timer4 time base strukture
		TIM_TimeBaseStructure.TIM_Prescaler = 2999; 																// 84MHz/(1/3.5ms)= (psc+1)*(arr+1)
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 							// counts from 0 to autoreload, and then back to 0
		TIM_TimeBaseStructure.TIM_Period = 65535; 
		TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0; 
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
		
		// enable hall sensor interface
		// T1F_ED (edge detector) will be connected to  HallSensor Inputs 
		// - T1F_ED je prikljucen na ulaz i detektira promjenu brida
		// TIM4_CH1,TIM4_CH2,TIM4_CH3 
		// Enables or disables the TIMx's Hall sensor interface.
		TIM_SelectHallSensor(TIM4, ENABLE);   																		
		
		// HallSensor event is delivered with singnal TI1F_ED - promjena stanja registra
		// odnosno pojava rastuceg ili padajuceg brida
		// (this is XOR of the three hall sensor lines)
		// Signal TI1F_ED: falling and rising ddge of the inputs is used 
		TIM_SelectInputTrigger(TIM4, TIM_TS_TI1F_ED); 
		
		// Channel_123 in input capture mode
		// When an input capture occurs:
		// The TIMx_CCR1 register gets the value of the counter on the active transition.
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; 
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge; 							// na rastuci i padajuci brid
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_TRC;									// listen to T1F, the  HallSensorEvent 
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;										 		// Div:1, capture is done every edge 
		TIM_ICInitStructure.TIM_ICFilter = 0x0; 																		// noise filter: 1111 
		
		TIM_ICInit(TIM4, &TIM_ICInitStructure); 
		
		// enable the interrupt for timer4
		// TIM capture compare 1 interrupt source
		TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE); 							
		
		
		// interrupt configuration
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_Cmd(TIM4, ENABLE);
}

volatile uint16_t kut_rotora;
kut rotor;
HallSensors Hall;

void TIM4_IRQHandler(void) {  
	
				Hall.HALL1 = GPIO_ReadInputDataBit( GPIOD, GPIO_Pin_12);										// cita inpute sa senzora i stavlja ih low ili high state
				Hall.HALL2 = GPIO_ReadInputDataBit( GPIOD, GPIO_Pin_13);										
				Hall.HALL3 = GPIO_ReadInputDataBit( GPIOD, GPIO_Pin_14);
				
				if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET ){
					TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);																	// brisanje zastavice za zahtjev prekida da ne ulazi dva puta u isti
					NVIC_ClearPendingIRQ(TIM4_IRQn);																					// prekid jer se ne stigne izbrisat dok prekid traje
					
					Hall.time = TIM4->CNT;																											// vrijednost cnt registra koji se ovjezi nakon svakog brida
					
					if (Hall.HALL1 == 1 && Hall.HALL2 == 0 && Hall.HALL3 == 1 )								// provjerava jel ulaz koji je izazvao prekid u low ili high state
						kut_rotora = rotor.th300;
					else if (Hall.HALL1 == 0 && Hall.HALL2 == 1 && Hall.HALL3 == 0 )
						kut_rotora = rotor.th120;	
					else if (Hall.HALL1 == 1 && Hall.HALL2 == 1 && Hall.HALL3 == 0 )
						kut_rotora = rotor.th60;
					else if (Hall.HALL1 == 0 && Hall.HALL2 == 0 && Hall.HALL3 == 1 )
						kut_rotora = rotor.th240;		
					else if (Hall.HALL1 == 0 && Hall.HALL2 == 1 && Hall.HALL3 == 1 )
						kut_rotora = rotor.th180;
					else 
						kut_rotora = rotor.th0;
				}
}

void Angle_Calibratet(kut theta){
		rotor = theta;
}

uint16_t GetKut(void){
    return kut_rotora;																																	// vraca izmjerenu brzinu, brzina w je globalna varijabla
}

uint32_t GetTime(void){
    return Hall.time; 																																	// vraca timer counter izmedu dva brida
}
