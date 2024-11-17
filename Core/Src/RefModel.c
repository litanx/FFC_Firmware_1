/*
 * RefModel.c
 *
 *  Created on: Oct 28, 2024
 *      Author: diego
 */

#include "RefModel.h"

#include "StepperController.h"


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

}


// mass-spring-damper model
// xdd = 1/m [f(t) - c*xd - k*x]
// Compute system position for a given input force

void refModel_Tick(rMod_t *mod, float inputForce){

	double dt = (double)mod->dt / 1000000;	// Convert dt to (s)

	// Compute ref Velocity
	mod->vel = mod->vel_1 + (dt * mod->acc_1);

	// Compute ref Position
	mod->pos = mod->pos_1 + (dt * mod->vel_1);

	/* Here calculate forces relatives to the velocity (i.e. friction, damping, etc) */

	/* Here calculate forces relative to the position of the system (i.e. variable spring K) */
	float springForce = interpolate_force(mod, (StepCon_GetPosition()/1000));

	/* Do I want to have mass dependent to the position? for instance I could emulate backslash */
	/* Do I want to have damping and friction dependent to the position? emulate different surfaces? */
	/* In a two axis controller the forces relatives to position will depend on a 2 dimensional array */

	// Compute ref Acceleration ->  ∑F = m * a
	//mod->acc = ((1 / (mod->m)) * (inputForce - (mod->c * mod->vel) - (mod->k * mod->pos)));
	mod->acc = ((1 / (mod->m)) * (inputForce - (mod->c * mod->vel) - springForce ));

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

//	float curve[][2] = {{-0.10, -50}, {-0.10, 0}, {0.02, 0}, {0.03, 20}, {0.03, -20}, {0.04, 0}, {0.10, 0}, {0.10, 50}};
	float curve[][2] = {{-0.10, -50}, {-0.10, -20}, {0.0, -5}, {0.0, 5}, {0.1, 20}, {0.1, 20}, {0.10, 200}, {0.10, 50}};


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



