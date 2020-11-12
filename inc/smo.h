#ifndef SMO_H
#define SMO_H

#include "userdefine.h"
/* =================================================================================
File name:       SMOPOS.H

Originator:	Digital Control Systems Group
			Sliding mode control

Description:
Header file containing constants, data type definitions, and
function prototypes for the .
=====================================================================================*/
typedef struct
{
    struct
    {
        s16 swValpha; // Input: Stationary alfa-axis stator voltage
        s16 swVbeta;  // Input: Stationary beta-axis stator voltage
        s16 swIalpha; // Input: Stationary alfa-axis stator current
        s16 swIbeta;  // Input: Stationary beta-axis stator current
    } inputs;
    struct
    {
        s16 swZalpha;         // Output: Stationary alfa-axis sliding control
        s16 swZbeta;          // Output: Stationary beta-axis sliding control
        s16 swEalphaFiltered; // Variable: Stationary alfa-axis back EMF
        s16 swEbetaFiltered;  // Variable: Stationary beta-axis back EMF
        s16 swTheta;          // Output:  rotor angle
        s16 swPreTheta;       // Output: Previous rotor angle
        s16 swAccTheta;       // Output: accumulated angle of a speed control loop
    } outputs;
    s16 swGsmopos;      // Parameter: Motor dependent control gain
    s16 swFsmopos;      // Parameter: Motor dependent plant matrix
    s16 swEalpha;       // Variable: Stationary alfa-axis back EMF
    s16 swEstIalpha;    // Variable: Estimated stationary alfa-axis stator current
    s16 swEbeta;        // Variable: Stationary beta-axis back EMF
    s16 swEstIbeta;     // Variable: Estimated stationary beta-axis stator current
    s16 swIalphaError;  // Variable: Stationary alfa-axis current error
    s16 swKCoeff;       // Parameter: Sliding control gain
    s16 swKslide;       // Parameter: Sliding control gain
    s16 swIbetaError;   // Variable: Stationary beta-axis current error
    s16 swKslf;         // Parameter: Sliding control filter gain
    s16 swKslfMin;      // Parameter: Sliding control filter gain
    s16 swOmegKslf;     // Parameter: Sliding control filter gain
    s16 swOmeg;         //It is not really speed, it is the incremeted angle in a speed loop time
    s16 swOmegfiltered; //It is not really speed, it is the incremeted angle in a speed loop time
    s16 swOmeg2RPM_Coeff;
    s32 slOmegTemp;
    s32 slOmegTempFiltered;
    s16 swMaxSMCErr;
    u8 ubAccumThetaCnt;
    u8 isTimeToRun;
} SMOPOS_OBJECT;

typedef enum
{
    E_RUN_UF,
    E_RUN_SWITCHING,
    E_RUN_FIX_SMO,
    E_RUN_FP_SMO
} E_RUN_STATE;

void SMO_Intialize(SMOPOS_OBJECT *v);
void SMOpos_calc(SMOPOS_OBJECT *v);
void fixsmo_speedpid(s16 SetpointValue);
void fixsmo_transfer(void);

extern SMOPOS_OBJECT smo1;
extern s16 swOmegConvCoeff;

void smo_vInit(float fRs, float fLs);
float smo_vCalc(float fIa, float fIb, float fUa, float fUb, float fOmega);
#endif // ! SMO_H