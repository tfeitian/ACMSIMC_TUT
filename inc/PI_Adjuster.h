/* =================================================================================
File name:       PI_Adjuster.H  (IQ version)

Originator:	Digital Control Systems Group
			Texas Instruments

Description:
Header file containing constants, data type definitions, and
function prototypes for the PIDREG3.
=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
------------------------------------------------------------------------------*/
#ifndef __PI_Adjuster_H__
#define __PI_Adjuster_H__

#include "userdefine.h"

typedef struct
{
    struct
    {
        volatile s16 Ref; // Input: Reference input
        s16 Fdb;          // Input: Feedback input
        s16 Kp;           // Parameter: Proportional gain
        s16 Ki;           // Parameter: Integral gain
        s16 Kc;           // Parameter: Integral correction gain
    } inputs;

    struct
    {
        s32 Out; // Output: PID output
    } outputs;

    s32 Err;       // Variable: Error
    s64 Up;        // Variable: Proportional output
    s64 Ui;        // Variable: Integral output
    s64 OutPreSat; // Variable: Pre-saturated output
    s32 OutMax;    // Parameter: Maximum output
    s32 OutMin;    // Parameter: Minimum output
    s64 SatErr;
} PIDREG_OBJECT;

typedef PIDREG_OBJECT *PIDREG_handle;
/*-----------------------------------------------------------------------------
Default initalizer for the PIDREG object.
-----------------------------------------------------------------------------*/
#define PIDREG_DEFAULTS          \
    {                            \
        {0, 0, 0, 0, 0},         \
            {0},                 \
            0, 0, 0, 0, 0, 0, 0, \
    }
/*------------------------------------------------------------------------------
Prototypes for the functions in PIDREG.C
------------------------------------------------------------------------------*/
void PidReg_Reset(PIDREG_OBJECT *v);
void PidReg_Calculate(PIDREG_handle);
void PidReg_Intialize(PIDREG_OBJECT *v);

extern PIDREG_OBJECT IdRegulate;
extern PIDREG_OBJECT IqRegulate;
extern PIDREG_OBJECT SpeedRegulate;

#define HIGH_PI_CURRENT_GAIN     \
    IdRegulate.inputs.Kp = 4096; \
    IdRegulate.inputs.Ki = 4000; \
    IqRegulate.inputs.Kp = 4096; \
    IqRegulate.inputs.Ki = 4000;
#define LOW_PI_CURRENT_GAIN     \
    IdRegulate.inputs.Kp = 512; \
    IdRegulate.inputs.Ki = 512; \
    IqRegulate.inputs.Kp = 512; \
    IqRegulate.inputs.Ki = 512;

#define PI_SHIFT_CNT 0

#define MAX_UI ((s64)30000 << PI_SHIFT_CNT) //983040000) //((long)30000 << 15)
#endif                                      // __PIDREG3_H__
