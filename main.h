/* Main.h */

#ifndef MAIN_H
#define MAIN_H

#include <stm32f4xx.h> 				// common stuff
#include <stm32f4xx_gpio.h> 	// gpio control
#include <stm32f4xx_rcc.h> 		// reset anc clocking
#include <stm32f4xx_tim.h> 		// timer
#include <stm32f4_discovery.h>
#include <HALL_timer_config.h>
#include <kalman.h>
#include <usart.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

typedef struct {
			double th0;
			double th60;
			double th120;
			double th180;
			double th240;
			double th300;
} kut;

void vInit_system(void *pvParameters);
void vKalibracija(void *pvParameters);
void vKalmanTask(void *pvParameters);
kut GetTheta(void);

#endif
