#include <math.h>
#include "kalman_c.h"

float Kalmanfilter(float inputNum, kalman_state* kstate){

	if (isnan(kstate->x) || isnan(kstate->p) || isinf(kstate->x) || isinf(kstate->p) || (kstate->p + kstate->r==0)){
		return -1;
	}
	kstate->p = kstate->p + kstate->q;
	kstate->k = kstate->p/(kstate->p + kstate->r);
	kstate->x = kstate->x + kstate->k * (inputNum - kstate->x);
	kstate->p = (1 - kstate->k) * kstate->p;

	return kstate->x;
}

