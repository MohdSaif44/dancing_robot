
/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"
#include "math.h"

void set(void) {

	Initialize();
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	PSxSlaveInit(&ps4, &hi2c1);
//	PSxInitDMA(&ps4,&hi2c1);
	TIMxInit(&htim6, 20000, 84, 5, 0); //20ms
	TIMxInit(&htim7, 5000, 84, 6, 0); //5ms
	RNS_config(&hcan1);
	MODNRobotBaseInit(MODN_FWD_OMNI, 1.0, 1.0, &Modn);
	MODNRobotVelInit(&xr, &yr, &wr, &Modn);
	MODNWheelVelInit(&v1, &v2, &v3, &v4, &Modn);
//	PIDSourceInit(in, out, pid);/

}


void RNS_config(CAN_HandleTypeDef* hcanx) {

	RNSInit(hcanx, &rns);
	//Encoder dcba(0-swap, 1-keep)  BDC dcba(0-keep, 1-swap) //0x00 0x00 0x
	RNSSet(&rns, RNS_DEVICE_CONFIG, (float) 0b0000001, (float) fwd_omni, (float) roboconPID);
	RNSSet(&rns, RNS_X_Y_ENC_CONFIG, 0.05 / 4000 * 3.142, 1.0, 0.05 / 4000 * 3.142, 1.0); //1.0 for nonswap , 2.0 for swap
	RNSSet(&rns, RNS_F_KCD_PTD, 5226.0/ 5192.0, (float)(0.125 * 3.142 / 522.6));
	RNSSet(&rns, RNS_B_KCD_PTD, 5174.0/ 5300.0, (float)(0.125 * 3.142 / 517.4));

	RNSSet(&rns, RNS_F_LEFT_VEL_SATEU, 1.0, 1.0 / 6.88, 19999.0);
	RNSSet(&rns, RNS_F_RIGHT_VEL_SATEU, 1.0, 1.0 / 8.62, 19999.0);
	RNSSet(&rns, RNS_B_LEFT_VEL_SATEU, 1.0, 1.0 / 7.337, 19999.0);
	RNSSet(&rns, RNS_B_RIGHT_VEL_SATEU, 1.0, 1.0 / 7.539, 19999.0);

	RNSSet(&rns, RNS_F_LEFT_VEL_PID,  4.10, 1.00, 0.003);
	RNSSet(&rns, RNS_F_RIGHT_VEL_PID, 4.10, 1.00, 0.003);
	RNSSet(&rns, RNS_B_LEFT_VEL_PID,  4.60, 1.00, 0.004);
	RNSSet(&rns, RNS_B_RIGHT_VEL_PID, 4.60, 1.000, 0.003);

	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_BASE, 0.2, 0.2, 0.2);
	RNSSet(&rns, RNS_F_LEFT_VEL_FUZZY_PID_PARAM, 0.02, 0.02, 0.02);

	RNSSet(&rns, RNS_PPInit); //Path Planning
	RNSSet(&rns, RNS_PPPathPID, 1.0, 0.5, 0.5);
	RNSSet(&rns, RNS_PPEndPID, 0.5, 0.1, 0.7);
	RNSSet(&rns, RNS_PPZPID, 1.0, 0.05, 0.2, 5.5);
	RNSSet(&rns, RNS_PPSetCRV_PTS, 10.0);         // Change No. of Points in the Curved Path
}

