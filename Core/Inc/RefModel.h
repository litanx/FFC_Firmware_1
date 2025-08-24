/*
 * RefModel.h
 *
 *  Created on: Oct 28, 2024
 *      Author: diego
 */

#ifndef INC_REFMODEL_H_
#define INC_REFMODEL_H_

#include "stdint.h"

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

/* Generic PI Controller parameters */
typedef struct{

	/* Controller parameters */
	uint32_t dt;		// Sampling rate - time (us)

	float kp; 			// Proportional gain
	float ki;			// Integral gain

	float outMax; 		// Out max lim
	float outMin;		// Out min lim

	/* Controller status */
	float iTerm;		// Integral term

}piCon_t;



/* Characteristics map object */
typedef struct{

	float x;		// Linear position point
	float f;		// Force at this point

}cMap_1d_t;

// Reference model system parameters
typedef struct {

	uint32_t dt; 		// Sampling rate - time (us)

	/* Model parameters */

	float m; 			// Mass (Kg)

	//float k; 			// Elastic constant (N/m)
	uint16_t k_mapSize;// number of defined points in the characteristics map
	void* k_map; 		// Characteristics map array void ptr for 1d model [(mm), (N)]

	float c; 			// Damping ratio (N.s/m)
	uint16_t c_mapSize;// number of defined points in the characteristics map
	void* c_map; 		// Characteristics map array void ptr for 1d model [(mm), (N)]

	float us; 			// Static friction coefficient  0.8
	float ud;			// Dynamic friction coefficient 0.4
	float N;			// Normal force (Friction force: F = u * N)
	float dfv; 			// Dynamic Friction Velocity (mm/s) (Speed at which dynamic friction becomes constant)

	/* Model limits */
	float posMaxLim; 	// Max displacement limit (mm)
	float posMinLim; 	// Min displacement limit (mm)

	float velMaxLim; 	// Max velocity limit (mm/s)
	float velMinLim; 	// Min velocity limit (mm/s)

	/* Model status */
	float acc;			// Acceleration (mm/s2)
	float vel;			// Velocity (mm/s)
	float pos;			// Position (mm)

	/*TODO: Probably I don't need to keep these */
	float acc_1;		// Prev Acceleration (mm/s2)
	float vel_1; 		// Prev Velocity (mm/s)
	float pos_1;		// Prev Position (mm)

	uint8_t vSaturated;		/* Flag Saturated Velocity - Prevent reaching Hardware limits*/
	uint8_t pSaturated;		/* Flag Saturated Position - Hard Stops Emulation */


}rMod_t;


/* Exported functions -------------------------------------------*/
float Compute_PI(piCon_t *con, float setpoint, float input);
void refModel_Tick(rMod_t *mod, double iForce);


#endif /* INC_REFMODEL_H_ */
