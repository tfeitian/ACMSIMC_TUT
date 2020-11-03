#include "AngleObserver.h"
#include "Fixmath.h"
#include "config.h"
#include "Fixpoint.h"

void AngleObserverInitalize(ANGLE_OBSERVER_OBJECT *p)
{
    p->swEstSin = 0;
    p->swEstCos = 0;
    p->swError = 0;
    p->swOmeg = 0;
    p->swAcc2 = 0;
    p->outputs.uwEstTheta = 0;

    p->swK1 = (s16)((float)800.0 * 800.0 * (float)T_SAMPLE * 32768.0 / (2 * Fre_MAX)); //Ts*K1/(2*fmax)    K1=wn*wn    here choose Wn=800
    p->swK2 = (s16)((float)3.0 * 2 * Fre_MAX * 4096 / 800);                            //2*fmax*K2 Q12//Q12
    p->swCoff = (s16)(2.0 * Fre_MAX * 32768.0 * (float)T_SAMPLE);                      //2*fmax*Ts*32768

    // p->swMaxStep=(s16)((float)OMEG_MAX*32768/20000);
}
void AngleObserverCalculate(ANGLE_OBSERVER_OBJECT *p)
{
    Trig_Components TrigCon1;
    p->swOmeg = p->swOmeg + (long)p->swError * p->swK1 / 32768;
    p->swAcc2 = p->swAcc2 + (long)p->swOmeg * p->swCoff / 32768; //2*fmax*Ts*32768
    p->outputs.uwEstTheta = (long)p->swOmeg * p->swK2 / 4096 + p->swAcc2;
    TrigCon1 = Trig_FunctionsAngle(p->outputs.uwEstTheta);
    p->swError = (long)p->inputs.swVsin * TrigCon1.hCos / 32768 - (long)p->inputs.swVcos * TrigCon1.hSin / 32768;
}