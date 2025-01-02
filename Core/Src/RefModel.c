/*
 * RefModel.c
 *
 *  Created on: Oct 28, 2024
 *      Author: diego
 */

#include <stdio.h>
#include <math.h>
#include "RefModel.h"

/*---------------------------------------------------------------------------------------*/
// Macros
// REDEFINED #define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))


/* Private functions --------------------------------------------------------------------*/
static float interpolate_force(rMod_t *mod, double x);


/*
 * NOTES:
 *
 * How to link two models
 *
 * How to implement additional external forces (i.e. vibration, etc) i.e. subsonic wav files
 *
 * How to implement a change of 0 ref (i.e. trim feature)
 *
 * How to implement backslash (i.e. a loose lever that has some play)
 *
 * */


//https://uk.mathworks.com/help/sps/ref/discretepicontroller.html
float Compute_PI(piCon_t *con, float setpoint, float input){

	float dt = (float)con->dt / 1000000;	// Convert dt to (s)

    // Calculate the error term
    float error = setpoint - input;

    // Calculate the integral term
    float temp_iTerm = con->iTerm + (con->ki * dt * error);

    // Calculate the output
    float u = (con->kp * error) + con->iTerm;

    // Saturate the output
    float u_sat = CONSTRAIN( u, con->outMin, con->outMax );

    if(u_sat == u) con->iTerm = temp_iTerm; 	// If output clipping, do not update controller iTerm

    return u_sat;

}


/*
 * Mass-spring-damper model with stick/slip friction
 * Compute system status for a given input force and position
 */

void refModel_Tick(rMod_t *mod, double iForce, double iPosition){

	double frictionForce = 0;
	uint8_t stuck = 0;			/* = 1 when velocity is under dynamic friction velocity threshold (dfv) */

	double dt = (double)mod->dt / 1000000;	// Convert dt to (s)

	// Compute ref Velocity
	mod->vel = mod->vel_1 + (dt * mod->acc_1);

	// Limit Velocity Hard Stops
	mod->vSaturated = 0;

	if(mod->vel > mod->velMaxLim){

		mod->vel = mod->velMaxLim;
		mod->vSaturated = 1;
	}

	if(mod->vel < mod->velMinLim){

		mod->vel = mod->velMinLim;
		mod->vSaturated = 1;
	}

	// Compute ref Position
	mod->pos = mod->pos_1 + (dt * mod->vel);

	// Limit position Hard Stops
	mod->pSaturated = 0;
	
	if(mod->pos >= mod->posMaxLim){

		mod->pos = mod->posMaxLim;
		mod->vel = 0;
		mod->pSaturated = 1;		
	}

	if(mod->pos <= mod->posMinLim){

		mod->pos = mod->posMinLim;
		mod->vel = 0;
		mod->pSaturated = 1;
	}

	/* Calculate forces relative to the position of the system */
	double springForce = interpolate_force(mod, mod->pos);
//	double springForce = (mod->k * mod->pos);

	/* Calculate damping force */
	double dampingForce = (mod->c * mod->vel);

	/* Friction Model --------------------------------------------------------------------------------*/
	// F = u * N -> where N is the Normal force between the moving object and the sliding surface.

	/* Velocity - Hit crossing (threshold) */
	if( mod->vel < mod->dfv && mod->vel > (-mod->dfv)) stuck = 1;

	if(stuck){

		int8_t sign = ((iForce - springForce) > 0) ? 1 : -1; 	// Get the sign of the applied force (input-spring)
		double modForce = fabs(iForce - springForce);			// Get the module of the applied force

		/* Choose the smallest force: [applied force] Vs [Static Friction] */
		frictionForce = (modForce < (mod->us * mod->N)) ? (sign) * modForce : (sign) * (mod->us * mod->N);

	}else{

		int8_t sign = (mod->vel > 0) ? 1 : -1;					// Get the sign of the velocity
		frictionForce = (sign) * (mod->ud * mod->N);			// Calculate dynamic friction
	}

	/*------------------------------------------------------------------------------------------------*/

	 if(mod->pos > mod->posMaxLim){ /* Just for debugging - Remove if not used */
		 asm("NOP");
	 }

	/* Do I want to have mass dependent to the position? for instance I could emulate backslash */
	/* Do I want to have damping and friction dependent to the position? emulate different surfaces? */
	/* In a two axis controller the forces relatives to position will depend on a 2 dimensional array */

	// Compute ref Acceleration ->  ∑F = m * a
	mod->acc = ((1 / (mod->m)) * (iForce - dampingForce - frictionForce - springForce ));

	// Store previous values
	mod->pos_1 = mod->pos;
	mod->vel_1 = mod->vel;
	mod->acc_1 = mod->acc;

}

/*
 * This function returns force depending on the position defined in the characteristics 2d array.
 *
 * Linear Interpolation (y) = y1 + [(x-x1) × (y2-y1)]/ (x2-x1)
 * Where (x1,y1) & (x2,y2) are coordinates
 * x is the point to perform interpolation
 * y is the interpolated value.
 * */
static float interpolate_force(rMod_t *mod, double x){

	if(!mod->cMap_size) 	return 0; // Empty vector
	if(mod->cMap == NULL) 	return 0; // No vector defined

	cMap_1d_t* cMap = (cMap_1d_t*)mod->cMap;
	uint8_t last = mod->cMap_size - 1;

	/* if pos < min known value > saturate */
	if(x < cMap[0].x)				return cMap[0].f;

	/* if pos > max known value > saturate */
	else if(x > cMap[last].x)		return cMap[last].f;

	/* otherwise find the adjacent upper and lower points in the array to interpolate */
	for(int i=0; i<last; i++){

		if( cMap[i].x <= x && cMap[i+1].x >= x)
			return  cMap[i].f + ((x - cMap[i].x) * (cMap[i+1].f - cMap[i].f)) / (cMap[i+1].x - cMap[i].x);

	}

	return 0; /* The program should never reach this line */
}
