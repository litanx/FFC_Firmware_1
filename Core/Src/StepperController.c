/*
 * StepperController.c
 *
 *  Created on: Oct 27, 2024
 *      Author: diego
 */


#include "StepperController.h"


extern TIM_HandleTypeDef htim2;


static uint16_t pulseTime = 0; /* Controls the time between pulses hence the speed of the motor */
static uint8_t dir = 0;
static int32_t stepCount = 0;

double MotorPos = 0;

static uint16_t stepLimit = 0; // anti-stall, prevents the motor running if the refModel is not ticking.

///* Set current actuator position as Zero plus a given offset */
//void StepCon_setZero(float offset){
//
//	if(offset > MAX_LIM) 			ActuatorPos = MAX_LIM;
//	else if(offset < MIN_LIM)   	ActuatorPos = MIN_LIM;
//	else 							ActuatorPos = offset;
//}


///* Offset the actuator´s zero position */
//void StepCon_offfetZero(float offset){
//
//	StepCon_SetLinear(ActuatorPos + offset, 10);
//	StepCon_setZero(0);
//}



///*
// * Move the linear actuator a given amount of mm
// * time in uS
// *
// * */
//void StepCon_SetLinear(float pos, float time){
//
//	float diff;
//
//	if(pos > MAX_LIM) 			diff = MAX_LIM - ActuatorPos;
//	else if(pos < MIN_LIM)   	diff = MIN_LIM - ActuatorPos;
//	else 						diff = pos - ActuatorPos;
//
//	StepCon_Rotate((365*diff)/ACTUATOR_PITCH, time);
//
//	ActuatorPos = ActuatorPos + diff;
//}

//
///*
// * Rotate the Stepper motor a given amount of degrees
// * time in uS
// *
// * */
//void StepCon_Rotate(float deg, float time){
//
//	uint32_t stp_disp = 0;
//
//	StepCon_ENABLE; 										// Enable Stepper
//
//	if(deg < 0){
//		StepCon_CCW; 										// CCW Direction
//		stp_disp = (uint32_t)((MICROSTEP * deg*(-1))/360); 	// Calculate Steps to desired position
//	}
//	else{
//		StepCon_CW;  										// CW Direction
//		stp_disp = (uint32_t)((MICROSTEP * deg)/360); 		// Calculate Steps to desired position
//	}
//
//	uint16_t pulse = (uint16_t)( (time * 1000)/stp_disp);
//
////	if(pulse<50)
//		pulse = 1;
//
//	for(uint32_t i = 0; i<stp_disp; i++){ 					// move motor using pulses
//		delay_us(pulse/2);
//		StepCon_PulseUp;
//		delay_us(pulse/2);
//		StepCon_PulseDown;
//	}
//
//}


/* Returns actuator position in mm */
float StepCon_GetPosition(){

	return MotorPos;
}

// set linear actuator to a speed of speed mm/s
void StepCon_Speed(float speed){

	if (speed < 0.0001 && speed > -0.0001) pulseTime = 0; // Dead band

	else if(speed < 0){
		dir = 0;
		StepCon_CCW; 			// CCW Direction
		pulseTime = -1000000/((speed/MM_PER_REV)* MICROSTEP);
	}
	else{
		dir = 1;
		StepCon_CW; 			// CW Direction
		pulseTime = 1000000/((speed/MM_PER_REV)* MICROSTEP);
	}

	stepLimit = 0;

}



/*
 * Timer callback routine
 **/
void StepCon_pulseTick(){



	if(!pulseTime){
		__HAL_TIM_SET_COUNTER(&htim2, 1000);
		return;
	}

	if(pulseTime<4){
		__HAL_TIM_SET_COUNTER(&htim2, 1000);
		asm("NOP");
		return;
	}

	__HAL_TIM_SET_COUNTER(&htim2, pulseTime);

	if(dir && MotorPos > MAX_MOTOR_LIM) return;
	if(!dir && MotorPos < MIN_MOTOR_LIM) return;

	if(stepLimit > 240) return;
	stepLimit++;

	/*Send pulse */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, SET);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, RESET);

	if(dir) stepCount++;		/* Track how many pulses have been sent to the stepper */
	else 	stepCount--;



	MotorPos = ((double)stepCount / MICROSTEP) * MM_PER_REV;
}
