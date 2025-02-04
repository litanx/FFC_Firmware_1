/*
 * StepperController.c
 *
 *  Created on: Oct 27, 2024
 *      Author: diego
 */


#include "StepperController.h"


extern TIM_HandleTypeDef htim2;

#define MIN_PULSE_TIME 	5

static uint16_t pulseTime = MIN_PULSE_TIME; /* Controls the time between pulses hence the speed of the motor */
static uint8_t dir = 0;
static int32_t stepCount = 0;
static float sDeadBand = 0;
double MotorPos = 0;

static uint16_t stepLimit = 0; // anti-stall, prevents the motor running if the refModel is not ticking.


/* Returns actuator position in mm */
float StepCon_GetPosition(){
	MotorPos = ((double)stepCount / MICROSTEP) * MM_PER_REV;
	return MotorPos;
}

// set linear actuator to a speed of speed mm/s
void StepCon_Speed(float speed){

	//if (speed < sDeadBand && speed > -sDeadBand) mStop = 1; // Dead band

	//else
	if(speed < 0){
		//mStop = 0;
		dir = 0;
		StepCon_CCW; 			// CCW Direction
		pulseTime = -4000/speed;
	}
	else{
		//mStop = 0;
		dir = 1;
		StepCon_CW; 			// CW Direction
		pulseTime = 4000/speed;
	}

	stepLimit = 0; // Reste motor watchdog

}



/*
 * Timer callback routine
 **/
extern uint16_t oddDrv;
void StepCon_pulseTick(){

	if(pulseTime < MIN_PULSE_TIME){
		__HAL_TIM_SET_COUNTER(&htim2, MIN_PULSE_TIME);
		pulseTime = MIN_PULSE_TIME;
		oddDrv++;
	}

	__HAL_TIM_SET_COUNTER(&htim2, pulseTime);

	if(dir && MotorPos > MAX_MOTOR_LIM) return;
	if(!dir && MotorPos < MIN_MOTOR_LIM) return;

	if(stepLimit++ > 500) {
		//BKPT;
		return;
	}

	/*Send pulse */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, SET);
	while(__HAL_TIM_GET_COUNTER(&htim2) > pulseTime);
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, RESET);

	if(dir) stepCount++;		/* Track how many pulses have been sent to the stepper */
	else 	stepCount--;

}
