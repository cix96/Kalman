// * Kalman_filter.c * //

#include <kalman.h>

void kalman_init(kalman *p_kalman) {
	
			p_kalman->sigma_w = 0.002;				// vrijednost procesnog šuma, varijance
			p_kalman->sigma_v  = 0.001;				// vrijednost mjerne pogreške
			
			p_kalman->angle = 0; 							// pocetna vrijednost za kut
			p_kalman->speed	 = 0;							// pocetna vrijednost brzine rotora
			
			p_kalman->P[0][0]  = 1;  					// na pocetku cemo pretpostaviti da su vrijednosti na dijagonali jednake 1, a ostale 0
			p_kalman->P[0][1]  = 0;
			p_kalman->P[1][0]  = 0;
			p_kalman->P[1][1]  = 1;

}

double kalman_angle_calc(kalman *p_kalman, double speed_mea, double dt) {
	    
	    double S; 				// nazivnik za racunanje kalman gaina
			double K[2]; 			// kalman gain
			double X[2]; 			// Predikcija
	
			// update
			// X(n) = A*X(n-1); A = [ 1 dt; 0 1]; X = [ theta; w]
			// P_ = A*P*A' + Rww; Rww = [ 0.5*dt^3 dt^2; dt^2 dt]*sigma_w
			// Jednadzbe su dobivene raspisivanjem matricnih jednadzbi
	
			X[0] = p_kalman->angle + dt*p_kalman->speed;
			X[1] = p_kalman->speed;
	
			p_kalman->P[0][0] += p_kalman->P[0][1]*dt + p_kalman->P[1][0]*dt + p_kalman->P[1][1]*dt*dt + 0.5*dt*dt*dt*p_kalman->sigma_w;
			p_kalman->P[0][1] += p_kalman->P[1][1]*dt + dt*dt*p_kalman->sigma_w;
			p_kalman->P[1][0] += p_kalman->P[1][1]*dt + dt*dt*p_kalman->sigma_w;
			p_kalman->P[1][1] += dt*p_kalman->sigma_w;
	
			// predict
			// K = P_*H'*inv(H*P_*H' + Rvv)
			// X = X(n) + K*(Z - H*X(n)); H = [ 0 1 ]
			// P = ( I - K*H)*P_
	
			S = p_kalman->P[1][1] + p_kalman->sigma_v;  								// racunanje nazivnika za kalkuaciju Kalman gaina P*H'*inv(H*P*H' + Rvv)
			
			K[0] = p_kalman->P[0][1]/S;																	// kalman gain
			K[1] = p_kalman->P[1][1]/S;
			
			
			p_kalman->angle = X[0] + K[0]*(speed_mea - X[1]);
			p_kalman->speed = X[1] + K[1]*(speed_mea - X[1]);
			
			p_kalman->P[0][0] -= K[0]*p_kalman->P[1][0];
			p_kalman->P[0][1] -= K[0]*p_kalman->P[1][1];
			p_kalman->P[1][0] -= K[1]*p_kalman->P[1][0];
			p_kalman->P[1][1] -= K[1]*p_kalman->P[1][1];
			
			return p_kalman->angle;              												 // vraca vrijednost od theta u trenutku n
}
