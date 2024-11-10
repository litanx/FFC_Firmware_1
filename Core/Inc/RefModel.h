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

	double m; 			// Mass (Kg)
	double c; 			// Damping ratio (N.s/m)
	double k; 			// Elastic constant (N/m)
	uint32_t dt; 		// Sampling rate - time (us)

	float posMaxLim; 	// Max displacement limit (m)
	float posMinLim; 	// Min displacement limit (m)

	float velMaxLim; 	// Max velocity limit (m/s)
	float velMinLim; 	// Min velocity limit (m/s)

	float accMaxLim; 	// Max acceleration limit (m/s2)
	float accMinLim; 	// Min acceleration limit (m/s2)

	double acc;			// Acceleration
	double vel;			// Velocity
	double pos;			// Position

	double acc_1;		// Prev Acceleration
	double vel_1; 		// Prev Velocity
	double pos_1;		// Prev Position


}rMod_t;


/* Exported functions -------------------------------------------*/

void refModel_Tick(rMod_t *mod, float force);


#endif /* INC_REFMODEL_H_ */
