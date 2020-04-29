/* Main.c */

#include <main.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

xSemaphoreHandle  	xBinarySemaphore;				// binarni semafor

//void vInit_system(void *pvParameters){
//	// radi nešto
//}

//void vKalibracija(void *pvParameters){
//	// radi nešto
//}


int main (void) {
	
		uint32_t dt;
		float speed_mea;
		float	angle;		
		char buff1[500];	
		kalman p_kalman;
		
		kalman_init( &p_kalman);
		USART2_Init();	
		timer4_init();
		
		xBinarySemaphore = xSemaphoreCreateBinary(); // create binary sempahore

//		if (xBinarySemaphore != NULL){

//			xTaskCreate(vInit_system, "Inicializacija sustava", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//			xTaskCreate(vKalibracija, "Kalibracija", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//			vTaskStartScheduler();
//		}
//		
		// izvodenje Kalman filtra 
		while(1){										
			dt = getTime();
			speed_mea = getSpeed();
			angle = kalman_angle_calc(&p_kalman,speed_mea,dt);
			sprintf(buff1, " Brzina motora: %f\n Kut theta: %f\n Proteklo vrijeme: %d\n ", speed_mea, angle, dt);
			send_message(buff1);
		}
}
