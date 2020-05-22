// * Kalman_filter.h * //

// Definiranje varijabli koje su potrebne za racunanje kuta pomocu Kalman filtra 
// te funkcije za inicijalizaciju filtra, proracun i postavljanje vrijednosti kuta

#ifndef KALMAN_H
#define KALMAN_H

#include <stm32f4xx.h>
#include <stdio.h>
#include <math.h>
#include <stm32f4_discovery.h>

typedef struct {
					double sigma_w;  									// procesni šum, varijanca
					double sigma_v;                 	// pogreška mjerenja
					double angle; 										// estimator kuta
					double speed; 										// estimator brzine rotora
					double P[2][2];										// kovarijancijska matrica
} kalman;

void kalman_init(void);																														// funkcija za inicijalizaciju varijabli na pocetku rada
																																															// upisuju se vrijednosti u varijable, angle0 je kut na kojem se
																																															// rotor nalati u t = 0
void kalman_predict(double dt);	
double kalman_update(double speed_mea);																// funkcija za proracun estimacije kuta
																																															// last_angle je zadnja predvidena vrijednost, speed_mea mjerena
																																															// brzina, dt je potreban za matricu stanja
				
#endif
