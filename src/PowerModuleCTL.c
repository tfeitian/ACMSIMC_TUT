/* Includes ------------------------------------------------------------------*/
#include "userdefine.h"
#include "PowerModuleCTL.h"

TIM_TypeDef TIM1Tmp;

TIM_TypeDef *TIM1 = &TIM1Tmp;
DRIVEPH_OBJECT Driver1 = {DRIVE_ENABLE, 1500};

void PWM_Initialize(void)
{
    Driver1.inputs.uwPwmPeriod = 1500;
    Driver1.inputs.ubEnable = DRIVE_DISABLE;
}

void PWM_Update(DRIVEPH_OBJECT *p)
{
    u32 temp2, temp3;
    if (p->inputs.ubEnable == DRIVE_ENABLE)
    {
#if 1
        if (p->inputs.uwTa < 131)
            p->inputs.uwTa = 0;
        if (p->inputs.uwTb < 131)
            p->inputs.uwTb = 0;
        if (p->inputs.uwTc < 131)
            p->inputs.uwTc = 0;

        if (p->inputs.uwTa > 32637)
            p->inputs.uwTa = 32767;
        if (p->inputs.uwTb > 32637)
            p->inputs.uwTb = 32767;
        if (p->inputs.uwTc > 32637)
            p->inputs.uwTc = 32767;
#endif

        if (1)
        {
            temp2 = p->inputs.uwTa * p->inputs.uwPwmPeriod;
            TIM1->CCR3 = (u16)(temp2 >> 15);
            temp2 = p->inputs.uwTb * p->inputs.uwPwmPeriod;
            TIM1->CCR2 = (u16)(temp2 >> 15);
            temp2 = p->inputs.uwTc * p->inputs.uwPwmPeriod;
            TIM1->CCR1 = (u16)(temp2 >> 15);
        }
        else
        {
            temp2 = p->inputs.uwTa * p->inputs.uwPwmPeriod;
            TIM1->CCR2 = (u16)(temp2 >> 15);
            temp2 = p->inputs.uwTb * p->inputs.uwPwmPeriod;
            TIM1->CCR3 = (u16)(temp2 >> 15);
            temp2 = p->inputs.uwTc * p->inputs.uwPwmPeriod;
            TIM1->CCR1 = (u16)(temp2 >> 15);
        }
    }
    else if (p->inputs.ubEnable == DRIVE_BRAKE) //75% brake
    {
        // temp3 = (u16)MotorStateMachine.uwBrakePercent;
        TIM1->CCR1 = temp3;
        TIM1->CCR2 = temp3;
        TIM1->CCR3 = temp3;
    }
    else
    {
        TIM1->CCR1 = p->inputs.uwPwmPeriod;
        TIM1->CCR2 = p->inputs.uwPwmPeriod;
        TIM1->CCR3 = p->inputs.uwPwmPeriod;
    }
}