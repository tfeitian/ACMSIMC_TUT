/* =================================================================================
File name:       SVGEN_DQ.H  (IQ version)

Originator:	Digital Control Systems Group
			Texas Instruments

Description:
Header file containing constants, data type definitions, and
function prototypes for the SVGEN_DQ.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
------------------------------------------------------------------------------*/
#ifndef __SVGEN_AB_H__
#define __SVGEN_AB_H__

typedef struct
{
	u8 ubSector;
	s32 Ualpha; // Input: reference alpha-axis phase voltage
	s32 Ubeta;	// Input: reference beta-axis phase voltage
	s32 Ta;		// Output: reference phase-a switching function
	s32 Tb;		// Output: reference phase-b switching function
	s32 Tc;		// Output: reference phase-c switching function
} SVGENAB;

#define HALF_SQRT_3 28378
#define T_PWM 32767
#define SVPWM_FIVE_SEGMENTS 0
#define SVPWM_SEVEN_SEGMENTS 1

// #define MIN_CURRENT_SMAPLE_INTERVAL (u16)((long long)3 * 65536.0 * PWM_FREQUENCY / 1000000) //3uS
// #define NO_ENOUGH_TIME2CURRENT_SAMPLE AdcResult.ubNoCurrentSample = 1
/*------------------------------------------------------------------------------
Prototypes for the functions in SVGEN_DQ.C
------------------------------------------------------------------------------*/
void Svpwm_Alpha_Belt_Initialize(SVGENAB *v);
void Svpwm_Alpha_Belt_Calc(SVGENAB *v);
extern SVGENAB sv1;
#define SVPWM_SECTOR sv1.ubSector
#endif // __SVGEN_DQ_H__
