/*
 * user_define.h
 *
 *  Created on: 2022年8月23日
 *      Author: a5073522
 */

#ifndef USER_DEFINE_H_
#define USER_DEFINE_H_

#define GOI_OLVO			1 	/* grid-off inverter open loop voltage output */
								/*
								 * Hardware Connection:
								 *  CON1 (DC connector): 150VDC input
								 *  CON2 (AC connector): R = 50ohm
								 * Result: After click "RUN" button on GUI, get 50V/50Hz AC on CON2
								 */
#define GOI_CL				2 	/* grid-off inverter current loop */
								/*
								 * Hardware Connection:
								 *  CON1 (DC connector): 150VDC input
								 *  CON2 (AC connector): R = 50ohm
								 * Result: After click "RUN" button on GUI, get 50V/50Hz AC on CON2
								 */
#define GOI					3 	/* grid-off inverter */
								/*
								 * Hardware Connection:
								 * 	CON1 (DC connector): 400VDC input
								 * 	CON2 (AC connector): R = 50ohm/1kW
								 * Result: After click "RUN" button on GUI, get 220V/50Hz AC on CON2
								 */
#define GTI_CL				4	/* grid-tied inverter current loop */
								/*
								 * Hardware Connection:
								 *  CON1 (DC connector): 400VDC input
								 *  CON2 (AD connector): 220V/50Hz AC
								 * Result: After click "RUN" button on GUI, get 3A/50Hz on CON2
								 */
#define PFC					5 	/* PFC */
								/*
								 * Hardware Connection:
								 *  CON1 (DC connector): R = 200ohm/1kW
								 *  CON2 (AC connector): 220VAC/50Hz input
								 * Result: After click "RUN" button on GUI, get 400VDC on CON1
								 */
#define NONE				6
#define DEBUG_PRO			NONE


#define M_PI                3.14159265358979323846
#define Ts                  0.0001 //the interval time of 10k timer(g_agt0_10kHz)

/* Error trigger and recovery threshold */
#define ERR_IAC_MAX         30
#define ERR_IAC_MIN         -30
#define ERR_VDC_Tri_H       420
#define ERR_VDC_Rcy_H       385
#if DEBUG_PRO < GOI
#define ERR_VDC_Tri_L       100
#else
#define ERR_VDC_Tri_L       350
#endif
#define ERR_VAC_RMS_MAX     264
#define ERR_VAC_RMS_MIN     80
#define ERR_FAC_MAX         65
#define ERR_FAC_MIN         45
#define ERR_TEMP_Tri_H      60
#define ERR_TEMP_Rcy_H      55
#define ERR_PAC_MAX         4000
#define ERR_PAC_CNT         (3/Ts)  //3s
#define ERR_PLL_RATE        0.02    //2%

/* Dataflash */
#define DATAFLASH_ADDR      0x08000000
#define DATAFLASH_BLOCK_NUM 1
#define DATAFLASH_BYTE      28

#endif /* USER_DEFINE_H_ */
