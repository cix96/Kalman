// * Kalman_filter.c * //

#include <kalman.h>

kalman p_kalman;
double S; 				// nazivnik za racunanje kalman gaina
double K[2]; 			// kalman gain

void kalman_init(void) {
	
			p_kalman.sigma_w = 0.002;				// vrijednost procesnog šuma, varijance
			p_kalman.sigma_v  = 0.001;				// vrijednost mjerne pogreške
			
			p_kalman.angle = 0; 							// pocetna vrijednost za kut
			p_kalman.speed	 = 0;							// pocetna vrijednost brzine rotora
			
			p_kalman.P[0][0]  = 1;  					// na pocetku cemo pretpostaviti da su vrijednosti na dijagonali jednake 1, a ostale 0
			p_kalman.P[0][1]  = 0;
			p_kalman.P[1][0]  = 0;
			p_kalman.P[1][1]  = 1;

}

void kalman_predict(double dt) {
	
			// update
			// X(n) = A*X(n-1); A = [ 1 dt; 0 1]; X = [ theta; w]
			// P_ = A*P*A' + Rww; Rww = [ 0.5*dt^3 dt^2; dt^2 dt]*sigma_w
			// Jednadzbe su dobivene raspisivanjem matricnih jednadzbi
	
			p_kalman.angle = p_kalman.angle + dt*p_kalman.speed;
			p_kalman.angle = fmod(p_kalman.angle, 360);
	
			p_kalman.P[0][0] += p_kalman.P[0][1]*dt + p_kalman.P[1][0]*dt + p_kalman.P[1][1]*dt*dt + 0.5*dt*dt*dt*p_kalman.sigma_w;
			p_kalman.P[0][1] += p_kalman.P[1][1]*dt + dt*dt*p_kalman.sigma_w;
			p_kalman.P[1][0] += p_kalman.P[1][1]*dt + dt*dt*p_kalman.sigma_w;
			p_kalman.P[1][1] += dt*p_kalman.sigma_w;
}

double kalman_update(double speed_mea){
			
			// predict
			// K = P_*H'*inv(H*P_*H' + Rvv)
			// X = X(n) + K*(Z - H*X(n)); H = [ 0 1 ]
			// P = ( I - K*H)*P_
	
			S = p_kalman.P[1][1] + p_kalman.sigma_v;  								// racunanje nazivnika za kalkuaciju Kalman gaina P*H'*inv(H*P*H' + Rvv)
			
			K[0] = p_kalman.P[0][1]/S;																	// kalman gain
			K[1] = p_kalman.P[1][1]/S;
			
			p_kalman.angle = p_kalman.angle + K[0]*(speed_mea - p_kalman.speed);
			p_kalman.speed = p_kalman.speed + K[1]*(speed_mea - p_kalman.speed);
			
			p_kalman.P[0][0] -= K[0]*p_kalman.P[1][0];
			p_kalman.P[0][1] -= K[0]*p_kalman.P[1][1];
			p_kalman.P[1][0] -= K[1]*p_kalman.P[1][0];
			p_kalman.P[1][1] -= K[1]*p_kalman.P[1][1];
			
			p_kalman.angle = fmod(p_kalman.angle, 360);
			
			return p_kalman.angle;              												 // vraca vrijednost od theta u trenutku n
}
