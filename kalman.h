// * Kalman_filter.h * //

// Definiranje varijabli koje su potrebne za racunanje kuta pomocu Kalman filtra 
// te funkcije za inicijalizaciju filtra, proracun i postavljanje vrijednosti kuta

#ifndef KALMAN_H
#define KALMAN_H

#include <stm32f4xx.h>
#include <stdio.h>
#include <stm32f4_discovery.h>
#include <HALL_timer_config.h>

typedef struct {
					double sigma_w;  								// procesni �um, varijanca
					double sigma_v;                 // pogre�ka mjerenja
					double angle; 									// estimator kuta
					double speed; 									// estimator brzine rotora
					double P[2][2];									// kovarijancijska matrica
} kalman;

void kalman_init(kalman *p_kalman);																														// funkcija za inicijalizaciju varijabli na pocetku rada
																																															// upisuju se vrijednosti u varijable, angle0 je kut na kojem se
																																															// rotor nalati u t = 0
double kalman_angle_calc(kalman *p_kalman, double speed_mea, double dt);											// funkcija za proracun estimacije kuta
																																															// last_angle je zadnja predvidena vrijednost, speed_mea mjerena
																																															// brzina, dt je potreban za matricu stanja
				
#endif
