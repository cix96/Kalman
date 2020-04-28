/* Main.c */

#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

xSemaphoreHandle  	xBinarySemaphore;				// binarni semafor

void vInit_system(void *pvParameters){
	// radi ne�to
}

void vKalibracija(void *pvParameters){
	// radi ne�to
}


int main (void) {
	
		double dt;
		double speed_mea;
		double angle;		
		char buff1[500];	
		kalman p_kalman;
		
		kalman_init( &p_kalman);
		USART2_Init();	
		timer4_init();
		
		xBinarySemaphore = xSemaphoreCreateBinary(); // create binary sempahore

		if (xBinarySemaphore != NULL){

			xTaskCreate(vInit_system, "Inicializacija sustava", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
			xTaskCreate(vKalibracija, "Kalibracija", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
			vTaskStartScheduler();
		}
		
		// izvodenje Kalman filtra 
		while(1){										
			dt = getTime();
			speed_mea = getSpeed();
			angle = kalman_angle_calc(&p_kalman,speed_mea,dt);
			sprintf(buff1, "Izmjerena brzina je %f , a kut theta %f\n", speed_mea, angle );
			send_message(buff1);
		}
}