/* Header hall_timer_config,h */

#ifndef TIMER_H
#define TIMER_H

#include <stm32f4xx.h> 								// common stuff
#include <stm32f4xx_gpio.h> 					// gpio control
#include <stm32f4xx_rcc.h> 						// reset anc clocking
#include <stm32f4xx_tim.h> 						// timer
#include <stm32f4_discovery.h>
#include <kalman.h>
#include <stdio.h>

void timer4_init(void);
void TIM4_IRQHandler(void);
double getSpeed(void);
double getTime(void);

double w;

#endif
