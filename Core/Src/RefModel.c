/*
 * RefModel.c
 *
 *  Created on: Oct 28, 2024
 *      Author: diego
 */

#include "RefModel.h"

	float posOffset = 0;
	float forceOffset = 0;
/*
 * NOTES:
 *
 *  How do we add friction??
 *
 * */

// mass-spring-damper model
// xdd = 1/m [f(t) - c*xd - k*x]
// Compute system position from a given input force

//void refModel_Tick(rMod_t *mod, float force){
//
////	float posOffset = 0;
////	float forceOffset = 0;
////
////	if(mod->pos < 0.03){
////		mod->k = 100;
////		posOffset = 0;
////		forceOffset = 0;
////	}else{
////		mod->k = 1000;
////		posOffset = 0.03;
////		forceOffset = 0.03*100;
////	}
//
//	double dt = (double)mod->dt / 1000000;	// Convert dt to (s)
//
//	// Compute Velocity
//	mod->vel = mod->vel_1 + (dt * mod->acc_1);
//
//	/* Apply limits only if they are defined */
////	if( mod->velMaxLim && mod->velMinLim ){
////		if(mod->vel  > mod->velMaxLim ) mod->vel  = mod->velMaxLim;
////		if(mod->vel  < mod->velMinLim ) mod->vel  = mod->velMinLim;
////	}
//
//	// Compute Position
//	mod->pos = mod->pos_1 + (dt * mod->vel_1);
//
//	/* Apply limits only if they are defined */
////	if( mod->posMaxLim && mod->posMinLim ){
////		if(mod->pos > mod->posMaxLim ) mod->pos = mod->posMaxLim;
////		if(mod->pos < mod->posMinLim ) mod->pos = mod->posMinLim;
////	}
//	// Compute Acceleration
//	mod->acc = (1 / mod->m) * (force -  (mod->c * mod->vel) - (mod->k * (mod->pos)));
//
//	/* Apply limits only if they are defined */
////	if( mod->accMaxLim && mod->accMinLim ){
////		if(mod->acc > mod->accMaxLim ) mod->acc = mod->accMaxLim;
////		if(mod->acc < mod->accMinLim ) mod->acc = mod->accMinLim;
////	}
//
//	// Store previous values	mod->pos_1 = mod->pos;
//	mod->vel_1 = mod->vel;
//	mod->acc_1 = mod->acc;
//
//}
//



void refModel_Tick(rMod_t *mod, float force){

//	float curve[][2] = {{-1,1000}, {-0.05 , 10000} ,{0,10}, {0.03,250},{0.04,-250}, {0.05,0}, {0.10,1000}, {1,1000}};
//
//	mod->k = 50;
//
//	for (int i = 0 ; i<8; i++){
//
//		if(mod->pos >= curve[i][0] && mod->pos < curve[i+1][0]){
//
//			posOffset = curve[i][0];
//			forceOffset = curve[i][0] * curve[i-1][1];
//			mod->k = curve[i][1];
//			break;
//		}
//	}

	double dt = (double)mod->dt / 1000000;	// Convert dt to (s)

	// Compute Velocity
	mod->vel = mod->vel_1 + (dt * mod->acc_1);

	// Compute Position
	mod->pos = mod->pos_1 + (dt * mod->vel_1);

	// Compute Acceleration
	mod->acc = ((1 / (mod->m)) * (force -  forceOffset - (mod->c * mod->vel) - (mod->k * (mod->pos - posOffset))));

	// Store previous values
	mod->pos_1 = mod->pos;
	mod->vel_1 = mod->vel;
	mod->acc_1 = mod->acc;

}
