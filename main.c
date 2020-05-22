/* Main.c */

#include <main.h>

xSemaphoreHandle  	xBinarySemaphore;				// binarni semafor
xTaskHandle xTaskHandle1, xTaskHandle2;

kut theta;
uint16_t fi, fi_p;
double t, t_p, dt;
double speed_mea;
double	angle;		

int main (void) {
		
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
			kalman_init();
			taskEXIT_CRITICAL();
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
	
		  Angle_Calibratet(theta);
			
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
				
				taskENTER_CRITICAL();
				t = GetTime();																													// dohvaca vrijednost cnt registra i trenutnog kuta
				fi = GetKut();
				taskEXIT_CRITICAL();
				if( t >= t_p) dt = t- t_p;
				else dt = 65535 + t - t_p;																							// ako slucajno dolazi do preljeva, odnosno reloada ARR registra pa nadoda cijeli period na to da bi se dobila prava vrijednost
				dt = (double)dt*(double)3000/42000000;																	// psc*period_izracunati/sys_clock
				
				
				if ( dt != 0){																													// ako je vrijednost dt 0 znaci da nije doslo do prekida
					if ( (fi_p == 300 || fi_p == 240)&& (fi == 0 || fi == 60))
							speed_mea = (double)(fi + 360 - fi_p)/dt;													// ispravljanje greške ako je prijelaz sa 300 na 0 da ne dode do krivog racuna
					else if ( (fi_p == 0 || fi_p == 60) && (fi == 300 || fi == 240))																// ista stvar samo ako motor ide u negativnu stranu
						speed_mea = (double)(fi - (fi_p + 360))/dt;
					else 
						speed_mea = (double)(fi- fi_p)/dt;
				}
				angle = kalman_update(speed_mea);
				for(int i = 0; i<3; i++){
						kalman_predict(dt);
				}
				fi_p  = fi;
				t_p = t;
			}
}
