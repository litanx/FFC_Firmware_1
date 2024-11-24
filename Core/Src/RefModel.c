/*
 * RefModel.c
 *
 *  Created on: Oct 28, 2024
 *      Author: diego
 */

#include "RefModel.h"
#include <math.h>

/* Private functions --------------------------------------------------------------------*/
static float interpolate_force(rMod_t *mod, double x);


/*
 * NOTES:
 *
 *  The force component due to the spring constant can be calculated by interpolating force values based
 *  on the position stored in an 2d array.
 *
 *  The friction can be done using a estate machine that uses the velocity as an input and the output value
 *  is added next to the damping coefficient. Just treat it as an additional force applied to the system.
 *
 * How to link two models
 *
 * How to implement aditional external forces (i.e. vibration, etc)
 *
 * How to implement a change of 0  ref (i.e. trim feature)
 *
 * How to implement backslash (i.e. a loose lever that has some play)
 *
 * */


//https://uk.mathworks.com/help/sps/ref/discretepicontroller.html

void posCont_Tick(pCon_t *con, double refPos, double realPos){

	/* Proportional controller */
	con->vel = con->Kp * ( refPos - realPos);

	//Limits
	if(con->vel > 0.8) con->vel = 0.8;
	if(con->vel < -0.8) con->vel = -0.8;

	/* TODO: Implement a  generic PI function that can be used multiple times using different struct variables */

}


/*
 * Mass-spring-damper model with stick/slip friction
 * Compute system status for a given input force and position
 */

void refModel_Tick(rMod_t *mod, double iForce, double iPosition){

	double frictionForce = 0;
	uint8_t saturated = 0;		/* Saturated Position - Hard Stops Emulation */
	uint8_t stuck = 0;			/* Velocity is under dynamic friction velocity threshold (dfv) */

	double dt = (double)mod->dt / 1000000;	// Convert dt to (s)

	// Compute ref Velocity
	mod->vel = mod->vel_1 + (dt * mod->acc_1);

	// Compute ref Position
	mod->pos = mod->pos_1 + (dt * mod->vel_1);

	// Limit position Hard Stops
	if(mod->pos > mod->posMaxLim){

		mod->pos = mod->posMaxLim;
		saturated = 1;
	}

	if(mod->pos < mod->posMinLim){

		mod->pos = mod->posMinLim;
		saturated = 1;
	}

	/* Calculate damping force */
	double dampingForce = (mod->c * mod->vel);

	/* Calculate forces relative to the position of the system */
//	double springForce = interpolate_force(mod, /*iPosition*/mod->pos);
	double springForce = (mod->k * mod->pos);

	/* Friction Model --------------------------------------------------------------------------------*/
	// F = u * N -> where N is the Normal force between the moving object and the sliding surface.

	/* Velocity - Hit crossing (threshold) */
	if( mod->vel < mod->dfv && mod->vel > (-mod->dfv)) stuck = 1;

	if(stuck){

		int8_t sign = ((iForce - springForce) > 0) ? 1 : -1;
		double modForce = fabs(iForce - springForce);

		/* Choose the smallest of these two*/
		frictionForce = (modForce < (mod->us * mod->N)) ? (sign)*modForce : (sign)*(mod->us * mod->N);

	}else{

		int8_t sign = (mod->vel > 0) ? 1 : -1;

		frictionForce = (sign) * (mod->ud * mod->N);
	}

	/*------------------------------------------------------------------------------------------------*/

	/* Do I want to have mass dependent to the position? for instance I could emulate backslash */
	/* Do I want to have damping and friction dependent to the position? emulate different surfaces? */
	/* In a two axis controller the forces relatives to position will depend on a 2 dimensional array */

	// Compute ref Acceleration ->  ∑F = m * a
	//mod->acc = ((1 / (mod->m)) * (inputForce - (mod->c * mod->vel) - (mod->k * mod->pos)));
	mod->acc = ((1 / (mod->m)) * (iForce - dampingForce - frictionForce - springForce ));

	// Reset Velocity integrator if required.
	if(saturated || stuck) 	mod->vel = 0;	// TODO: Do I need to reset if stuck? or can I let it run free?

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

//Sheet 3	//	float curve[][2] = {{-0.10, -50}, {-0.10, -20}, {-0.001, -5}, {0.001, 5}, {0.1, 20}, {0.1, 20}, {0.10, 20}, {0.10, 50}};
//Sheet 1
	float curve[][2] = {{-0.10, -50}, {-0.10, 0}, {0.028, 0}, {0.03, 7}, {0.03, -7}, {0.032, 0}, {0.10, 10}, {0.10, 50}};
//	float curve[][2] = {{-0.10, -50}, {-0.10, 0}, {0.02, 0}, {0.03, 20}, {0.03, -20}, {0.04, 0}, {0.10, 0}, {0.10, 50}};


	/* if pos < min known value saturate */
	if(x < curve[0][0])			return curve[0][1];

	/* if pos > max known value saturate */
	else if(x > curve[7][0])		return curve[7][1];

	/* otherwise find the adjacent upper and lower points in the array */
	for(int i=0; i<(8-1); i++){

		if( curve[i][0] <= x && curve[i+1][0] > x){

			return  curve[i][1] + ((x -curve[i][0]) * (curve[i+1][1]-curve[i][1])) / (curve[i+1][0]-curve[i][0]);

		}
	}

	return 0; /* The program should never reach this line */
}



