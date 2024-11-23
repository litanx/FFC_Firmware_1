/*
 * RefModel.h
 *
 *  Created on: Oct 28, 2024
 *      Author: diego
 */

#ifndef INC_REFMODEL_H_
#define INC_REFMODEL_H_

#include "stdint.h"

// Reference model system parameters
typedef struct {

	/* Model parameters */
	uint32_t dt; 		// Sampling rate - time (us)

	double m; 			// Mass (Kg)
	double c; 			// Damping ratio (N.s/m)
	double k; 			// Elastic constant (N/m) ** NOT USED ** Replaced by 2d array

	double us; 			// Static friction coefficient  0.8
	double ud;			// Dynamic friction coefficient 0.4
	double N;			// Normal force (Friction force: F = u * N)

	float posMaxLim; 	// Max displacement limit (m)
	float posMinLim; 	// Min displacement limit (m)

//	float velMaxLim; 	// Max velocity limit (m/s)
//	float velMinLim; 	// Min velocity limit (m/s)

//	float accMaxLim; 	// Max acceleration limit (m/s2)
//	float accMinLim; 	// Min acceleration limit (m/s2)

	/* Model status */

	double acc;			// Acceleration
	double vel;			// Velocity
	double pos;			// Position

	double acc_1;		// Prev Acceleration
	double vel_1; 		// Prev Velocity
	double pos_1;		// Prev Position


}rMod_t;


/* Position Controller parameters */
typedef struct{

	double Kp;		/* Proportional constant 	*/
	double Ki;		/* Integral constant 		*/

	double vel;		/* Velocity					*/
	double vel_1; 	/* Prev Velocity	 		*/

}pCon_t;

/* Exported functions -------------------------------------------*/

void posCont_Tick(pCon_t *con, double refPos, double realPos);
void refModel_Tick(rMod_t *mod, double iForce, double iPosition);


#endif /* INC_REFMODEL_H_ */
