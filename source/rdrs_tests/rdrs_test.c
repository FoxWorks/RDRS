#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define RDRS_LIBRARY
#include "rdrs.h"

void main() {
	int i;

//Frequencies
#define Hz		*1e-6
#define KHz		*1e-3
#define MHz		*1e0
#define GHz		*1e3
//Data rates
#define BPS		*8.0
#define KBPS	BPS*1024.0
#define MBPS	KBPS*1024.0
#define KBIT	*1.0e3
#define MBIT	*1.0e6
//Power
#define W		*1e0

	RDRS_CHANNEL ch1 = { RDRS_CHANNEL_QAM4, 
						435.00 MHz,
						RDRS_AUTO_BANDWIDTH, 
						128.0 KBPS };
	RDRS_CHANNEL ch2 = { RDRS_CHANNEL_QAM4, 
						435.00 MHz,
						RDRS_AUTO_BANDWIDTH, 
						128.0 KBPS };
	RDRS_ANTENNA* A;
	RDRS_ANTENNA* B;
	RDRS_ANTENNA* C;
	RDRS_SYSTEM* system;
	RDRS_System_Create(&system);

	/*{
		FILE* t = fopen("test.txt","w+");
		double SNR;
		for (SNR = -50; SNR < 30; SNR += 0.01) {
			double BER1,BER2,BER3,BER4;
			RDRS_AntennaPhysics_GetBER(RDRS_CHANNEL_QAM4,SNR,&BER1);
			RDRS_AntennaPhysics_GetBER(RDRS_CHANNEL_QAM16,SNR,&BER2);
			RDRS_AntennaPhysics_GetBER(RDRS_CHANNEL_QAM64,SNR,&BER3);
			RDRS_AntennaPhysics_GetBER(RDRS_CHANNEL_QAM256,SNR,&BER4);
			fprintf(t,"%f %f %f %f %f\n",SNR,BER1,BER2,BER3,BER4);
		}
		fclose(t);
		getc(stdin);
		exit(1);
	}*/

	//Create antennas
	RDRS_Antenna_Create(system,&A,ch2,RDRS_ANTENNA_NO_RECEIVE);
	RDRS_Antenna_Create(system,&B,ch1,RDRS_ANTENNA_NO_RECEIVE);
	RDRS_Antenna_Create(system,&C,ch1,RDRS_ANTENNA_NO_SEND);
	A->position.y = 1000e3;
	B->position.y = 1100e3;

	//RDRS_Antenna_SetSize(A,0.66);
	//RDRS_Antenna_SetSize(B,0.66);
	//RDRS_Antenna_SetSize(C,0.66);
	RDRS_Antenna_SetPower(A,10 W);
	RDRS_Antenna_SetPower(B,5 W);

	//Send some test data
	for (i = 0; i < 100; i++) {
		RDRS_Antenna_Send(A,0xAA);//(RDRS_SYMBOL)i);
	}
	for (i = 0; i < 100; i++) {
		RDRS_Antenna_Send(B,0x55);
	}

	//Transmission simulation
	RDRS_System_SetTime(system,1001.00);
	RDRS_Antenna_SimulateSend(A);
	RDRS_System_SetTime(system,1001.00);
	RDRS_Antenna_SimulateSend(B);
	RDRS_System_SetTime(system,1005.0);
	//RDRS_Antenna_SimulateReceive(B);
	C->read_time = 1001.00;
	RDRS_Antenna_SimulateReceive(C);	
}