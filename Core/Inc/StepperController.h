/*
 * StepperController.h
 *
 *  Created on: Oct 27, 2024
 *      Author: diego
 */

#ifndef INC_STEPPERCONTROLLER_H_
#define INC_STEPPERCONTROLLER_H_

#include "main.h"


/* Hardware ports definition ----------------------------------------------------*/

#define StepCon_ENABLE 		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)
#define StepCon_DISABLE 	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET)

#define StepCon_CCW			HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, RESET)
#define StepCon_CW 		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, SET)

#define StepCon_PulseUp 	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, SET);
#define StepCon_PulseDown 	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, RESET);

 /*-------------------------------------------------------------------------------*/

#define MICROSTEP   		10000		// Steps per rev
#define MM_PER_REV			40		// displacement per rev in mm
#define STEP_PWM			200 	// Time per pulse (uS)

#define ACTUATOR_PITCH		4		// linear displacement (mm/rev)

#define MAX_MOTOR_LIM 		110 		// Max displacement in mm
#define MIN_MOTOR_LIM		-110		// min displacement in mm


void delay_us(uint16_t us);



void StepCon_setZero(float offset);
void StepCon_offfetZero(float offset);

float StepCon_GetPosition();

void StepCon_SetLinear(float pos, float time);
void StepCon_Rotate(float deg, float time);
void StepCon_Speed(float speed);
void StepCon_pulseTick();

#endif /* INC_STEPPERCONTROLLER_H_ */
