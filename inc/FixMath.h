#ifndef __MATRIX_CONVERT_H__
#define __MATRIX_CONVERT_H__
#include "userdefine.h"

typedef struct
{
    s16 hCos;
    s16 hSin;
    u16 uwAngle;
} Trig_Components;
typedef struct
{
    struct
    {
        s16 swA; //Input phase A
        s16 swB; //Input phase B
    } inputs;
    struct
    {
        s16 swAlpha;
        s16 swBelt;
        s16 swDs;
        s16 swQs;
    } outputs;
    Trig_Components *pTrig;
} MATRIX_CONVERT;

typedef struct
{
    struct
    {
        s32 slDs; //Input phase A current
        s32 slQs; //Input phase B current
    } inputs;
    struct
    {
        s32 slAlpha;
        s32 slBelt;
    } outputs;
    Trig_Components *pTrig;
} IPARK;

#define ONE_SQRT_3 18919 //1/sqrt(3)

#define DEGREE60 10922
#define DEGREE90 16380
#define DEGREE180 32767
#define DEGREE270 49151
#define DEGREE360 65535

#define VDQ_DEFAULT          \
    {                        \
        {0, 0, 0}, {0, 0}, & \
    }

extern const u16 wSin_Table[];
extern const u16 uwActan_Table[];

void fp_abtodq(s16 ia, s16 ib, u16 u16theta, s16 *id, s16 *iq);
void fp_dqtoab(s32 ud, s32 uq, u16 theta, s32 *ua, s32 *ub);
s16 Math_Sin(u16 uwAngle);
s16 Math_Cos(u16 uwAngle);
#endif