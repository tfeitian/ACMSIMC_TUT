/*=====================================================================================
 File name:        PID_REG3.C

 Originator:

 Description:  The PID controller with anti-windup

=====================================================================================
-------------------------------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/
#include "PI_Adjuster.h"
#include "userdefine.h"

void PidReg_Reset(PIDREG_OBJECT *v)
{
    v->inputs.Fdb = 0;
    v->outputs.Out = 0;
    v->Err = 0;
    v->Up = 0;
    v->Ui = 0;
    v->OutPreSat = 0;
    v->SatErr = 0;
}
void PidReg_Intialize(PIDREG_OBJECT *v)
{
    v->inputs.Ref = 0;
    v->inputs.Fdb = 0;
    v->inputs.Kp = 128;       //Q9  *512
    v->inputs.Ki = 512;       //Q14 *16384
    v->inputs.Kc = 32768 / 8; //Q15 *32768
    v->outputs.Out = 0;
    v->Err = 0;
    v->Up = 0;
    v->Ui = 0;
    v->OutPreSat = 0;
    v->OutMax = 32760;
    v->OutMin = 2000;
    v->SatErr = 0;
}
void PidReg_Calculate(PIDREG_OBJECT *v)
{
    // Compute the error
    v->Err = (s32)(v->inputs.Ref - v->inputs.Fdb);
    // Compute the proportional output
    v->Up = (((s64)v->inputs.Kp * v->Err) >> 9);

    // Compute the integral output
    if ((v->Ui < MAX_UI) || (v->Ui > -MAX_UI))
    {
        v->Ui = (((((s64)v->Ui) << 15) + (((s64)v->inputs.Ki) * v->Err << 1) + (s64)v->inputs.Kc * v->SatErr) >> (15 - PI_SHIFT_CNT));
    }
    else if (v->Ui >= MAX_UI)
    {
        v->Ui = MAX_UI;
    }
    else if (v->Ui <= -MAX_UI)
    {
        v->Ui = -MAX_UI;
    }

    // Compute the pre-saturated output
    v->OutPreSat = (((v->Up << PI_SHIFT_CNT) + v->Ui) >> PI_SHIFT_CNT);

    // Saturate the output
    if (v->OutPreSat > v->OutMax)
        v->outputs.Out = v->OutMax;
    else if (v->OutPreSat < v->OutMin)
        v->outputs.Out = v->OutMin;
    else
        v->outputs.Out = v->OutPreSat;

    // Compute the saturate difference
    v->SatErr = ((s64)v->outputs.Out) - v->OutPreSat;
}
