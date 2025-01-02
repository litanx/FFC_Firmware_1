//https://hackernoon.com/how-to-set-up-c-debugging-in-vscode-using-a-makefile

#include <stdio.h>
#include "../Core/Inc/RefModel.h"

#define nSAMPLES 		2000
#define SAMPLERATE 		500 	//us 

int main(int argc, char const *argv[]) {

	printf("Program Start\n");
	
	// Export data
 	FILE *fp = NULL;

 	remove("plot.csv");
    fp = fopen("plot.csv" ,"a");

	if(fp == NULL) {
		printf("Failed to open plot.csv file\r\n");
		return -1;
	}
	

	fprintf(fp, "i,Force,Pos,vel,acc \n");	// CSV Header

	rMod_t hmod1 = {0};
	piCon_t hcon1 = {0}; // PI position controller
	cMap_1d_t curve[255] = {{-0.10, -10}, {0.10, 10}};

	hmod1.dt = SAMPLERATE; 	// us /* This can go lower than 500us due ADC timing limitations */

	hmod1.m = 0.5;
	hmod1.c = 10; 		// N.s/m
	hmod1.k = 50; 		// N/m

	hmod1.cMap = &curve;
	hmod1.cMap_size = 2;

	// hmod1.us = 0.2; 		// Dynamic friction coefficient
	// hmod1.ud = 0.2; 		// Static friction coefficient
	// hmod1.N = 1; 			// Normal Force (Weight)
	// hmod1.dfv = 0.00001;	// m/s

	hmod1.posMaxLim = 0.01; // Model Hard Stops
	hmod1.posMinLim = -0.01;

	hmod1.velMaxLim = 1;	// Hardware max reachable speed.
	hmod1.velMinLim = -1;

	hcon1.dt = hmod1.dt;
	hcon1.kp = 10;
	hcon1.ki = 1;
	hcon1.outMax = hmod1.velMaxLim;
	hcon1.outMin = hmod1.velMinLim;

	float Force = 20;

	for (int i=0; i<nSAMPLES; i++){

		//Get Force
		// Force = 1;
		if(i>200) Force = 0;

		// Reference model
		//------------------------------------------//
		 refModel_Tick(&hmod1, Force, 0);
		//------------------------------------------//

		// Position Controller
		//------------------------------------------//
		// float refSpeed = Compute_PI(&hcon1, hmod1.pos, (StepCon_GetPosition()/1000));
		fprintf(fp, "%d, %f,%f,%f,%f,%d,%f\n",i, Force, hmod1.pos*10, hmod1.vel, hmod1.acc, hmod1.pSaturated, hmod1.posMaxLim );
	}	


	fclose(fp);
	

	return 0;
}


