/* Main.c */

#include <main.h>

xSemaphoreHandle  	xBinarySemaphore;				// binarni semafor
xTaskHandle xTaskHandle1, xTaskHandle2;
kalman p_kalman;
kut theta;

double dt;
double speed_mea;
double	angle;		
char buff1[500];	

int main (void) {
		
		USART2_Init();
		timer4_init();
		
		xBinarySemaphore = xSemaphoreCreateBinary(); // create binary sempahore

		if (xBinarySemaphore != NULL){
			xTaskCreate(vInit_system, "Inicializacija sustava", configMINIMAL_STACK_SIZE, NULL, 1, &xTaskHandle2);
			xTaskCreate(vKalibracija, "Kalibracija", configMINIMAL_STACK_SIZE, NULL, 2, &xTaskHandle1);
			xTaskCreate(vKalmanTask, "Kalman filtar", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
			vTaskStartScheduler();
		}
		while(1);
}

void vInit_system(void *pvParameters){
			
			taskENTER_CRITICAL();
			kalman_init( &p_kalman);
			taskEXIT_CRITICAL();
			sprintf(buff1,"Sustav je inicijaliziran!\n");
			send_message(buff1);
			xSemaphoreGive(xBinarySemaphore);
			vTaskDelete(xTaskHandle2);
}

void vKalibracija(void *pvParameters){
			
			theta.th0 = 0;
			theta.th60 = 60;
			theta.th120 = 120;
			theta.th180 = 180;
			theta.th240 = 240;
			theta.th300 = 300;
			sprintf(buff1,"Sustav je kalibriran!\n");
			send_message(buff1);
			
			vTaskDelete(xTaskHandle1);
}

void vKalmanTask(void *pvParameters){
	
			// Ovaj Task je najveceg prioriteta i traži semafor koji je inicijalno zauzet i ceka ga beskonacno
			// nakon sto se semafor otpusti od strane taska nižeg prioriteta može pocet racunat kut
			// tasENTER_CRITICAL i taskEXIT_CRITICAL služe da se onemoguci prekid kada se racunaju ove dvije varijable
			// prije pocetka rada, ako nema prekida, ccr1 je 0 i dobije se beskonacna vrijednost za dt zato se provjerava da 
			// ne dode do krivog racunanja
			// nikad ne bi smjelo doci do i++
			xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
			while(1){
				if(1/getTime()!= 0){
				taskENTER_CRITICAL();
				dt = getTime();
				speed_mea = getSpeed();
				taskEXIT_CRITICAL();
				angle = kalman_angle_calc(&p_kalman,speed_mea,dt);
				sprintf(buff1,"w = %lf\ntheta = %lf\ndt = %lf\n", speed_mea, angle, dt);
				send_message(buff1);
				}
				else {
					int i;
					i++;
				}
			}
}

kut GetTheta(void){                                      	// prijenos varijable u interrupt
	return theta;
}
