#ifndef __ANGLE_OBSERVER_H__
#define __ANGLE_OBSERVER_H__
#include "userdefine.h"
typedef struct
{
    struct
    {
        s16 swVsin;
        s16 swVcos;
    } inputs;
    struct
    {
        u16 uwEstTheta; //
    } outputs;
    s16 swEstSin; //
    s16 swEstCos; //
    s16 swError;  //
    s16 swOmeg;   //
    s16 swAcc2;
    s16 swK1;
    s16 swK2;
    s16 swMaxStep;
    s16 swCoff;
} ANGLE_OBSERVER_OBJECT;
extern ANGLE_OBSERVER_OBJECT AngleObserver;
void AngleObserverInitalize(ANGLE_OBSERVER_OBJECT *p);
void AngleObserverCalculate(ANGLE_OBSERVER_OBJECT *p);
#endif