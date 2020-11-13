#include "FixMath.h"

#define SIN_TABLE_LENGTH 721
#define SIN_CAL_INDEX (SIN_TABLE_LENGTH - 1)
#define SIN_TABLE_SHIFT 10

#if 1

#endif

void Trig_Functions(Trig_Components *p)
{
    u16 hindex = 0;
    if (p->uwAngle < DEGREE90)
    {
        hindex = ((u32)p->uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = wSin_Table[hindex];
        p->hCos = wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (p->uwAngle < DEGREE180)
    {
        p->uwAngle = DEGREE180 - p->uwAngle;
        hindex = ((u32)p->uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = wSin_Table[hindex];
        ;
        p->hCos = -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (p->uwAngle < DEGREE270)
    {
        p->uwAngle = p->uwAngle - DEGREE180;
        hindex = ((u32)p->uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = -wSin_Table[hindex];
        p->hCos = -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else
    {
        p->uwAngle = DEGREE360 - p->uwAngle;
        hindex = ((u32)p->uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = -wSin_Table[hindex];
        p->hCos = wSin_Table[SIN_CAL_INDEX - hindex];
    }
}
/*******************************************************************************
* Function Name  : Trig_Functions
* Description    : This function returns Cosine and Sine functions of the input
*                  angle
* Input          : angle in u16 format
* Output         : Cosine and Sine in s16 format
* Return         : none.
*******************************************************************************/
#if 1
Trig_Components Trig_FunctionsAngle(u16 uwAngle)
{
    u16 hindex = 0;

    Trig_Components Local_Components; //

    if (uwAngle < DEGREE90)
    {
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = wSin_Table[hindex];
        Local_Components.hCos = wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (uwAngle < DEGREE180)
    {
        uwAngle = DEGREE180 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = wSin_Table[hindex];
        Local_Components.hCos = -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (uwAngle < DEGREE270)
    {
        uwAngle = uwAngle - DEGREE180;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = -wSin_Table[hindex];
        Local_Components.hCos = -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else
    {
        uwAngle = DEGREE360 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = -wSin_Table[hindex];
        Local_Components.hCos = wSin_Table[SIN_CAL_INDEX - hindex];
    }
    return (Local_Components);
}
#endif

s16 Math_Sin(u16 uwAngle)
{
    u16 hindex = 0;

    if (uwAngle < DEGREE90)
    {
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return wSin_Table[hindex];
    }
    else if (uwAngle < DEGREE180)
    {
        uwAngle = DEGREE180 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return wSin_Table[hindex];
    }
    else if (uwAngle < DEGREE270)
    {
        uwAngle = uwAngle - DEGREE180;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return -wSin_Table[hindex];
    }
    else
    {
        uwAngle = DEGREE360 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return -wSin_Table[hindex];
    }
    return 0;
}

s16 Math_Cos(u16 uwAngle)
{
    u16 hindex = 0;

    if (uwAngle < DEGREE90)
    {
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (uwAngle < DEGREE180)
    {
        uwAngle = DEGREE180 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (uwAngle < DEGREE270)
    {
        uwAngle = uwAngle - DEGREE180;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else
    {
        uwAngle = DEGREE360 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> SIN_TABLE_SHIFT;
        return wSin_Table[SIN_CAL_INDEX - hindex];
    }
    return 0;
}
#if 1
/*******************************************************************************
* Function Name  : Trig_Functions
* Description    : This function returns atan functions of the input
*                  value
* Input          : x,y in s16 format
* Output         : angle in u16 format
* Return         : none.
*******************************************************************************/
u16 Atan_Functions(s16 x, s16 y)
{
    u16 AbsX, AbsY;
    u16 uwIndex;
    u16 uwReturnValue;
    if (x > 0 && y > 0) //�ڵ�һ����
    {
        AbsX = x;
        AbsY = y;
        if (AbsX >= AbsY) //0~45��
        {
            //uwIndex=(long)(AbsY*32768)/AbsX;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsY * 512) / AbsX;
            uwReturnValue = uwActan_Table[uwIndex];
        }
        else //45~90��
        {
            //uwIndex=(long)(AbsX*32768)/AbsY;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsX * 512) / AbsY;
            uwReturnValue = 16384 - uwActan_Table[uwIndex];
        }
    }
    else if (x < 0 && y > 0) //�ڵڶ�����
    {
        AbsX = 0 - x;
        AbsY = y;
        if (AbsX >= AbsY) //0~45��
        {
            //uwIndex=(long)(AbsY*32768)/AbsX;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsY * 512) / AbsX;
            uwReturnValue = 32768 - uwActan_Table[uwIndex];
        }
        else //45~90��
        {
            //uwIndex=(long)(AbsX*32768)/AbsY;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsX * 512) / AbsY;
            uwReturnValue = 16384 + uwActan_Table[uwIndex];
        }
    }
    else if (x < 0 && y < 0) //�ڵ�������
    {
        AbsX = 0 - x;
        AbsY = 0 - y;
        if (AbsX >= AbsY) //0~45��
        {
            //uwIndex=(long)(AbsY*32768)/AbsX;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsY * 512) / AbsX;
            uwReturnValue = 32768 + uwActan_Table[uwIndex]; //pi+
        }
        else //45~90��
        {
            //uwIndex=(long)(AbsX*32768)/AbsY;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsX * 512) / AbsY;
            uwReturnValue = 49152 - uwActan_Table[uwIndex]; //1.5pi-
        }
    }
    else if (x > 0 && y < 0) //�ڵ�������
    {
        AbsX = x;
        AbsY = 0 - y;
        if (AbsX >= AbsY) //0~45��
        {
            //uwIndex=(long)(AbsY*32768)/AbsX;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsY * 512) / AbsX;
            uwReturnValue = 65535 - uwActan_Table[uwIndex];
        }
        else //45~90��
        {
            //uwIndex=(long)(AbsX*32768)/AbsY;
            //uwIndex=uwIndex/64;
            uwIndex = (long)(AbsX * 512) / AbsY;
            uwReturnValue = 49152 + uwActan_Table[uwIndex];
        }
    }
    else if (x == 0 && y > 0) //��y���������ᣬ����90��
        uwReturnValue = 16384;
    else if (x == 0 && y < 0) //��y���ϸ����ᣬ����270��
        uwReturnValue = 49152;
    else if (x > 0 && y == 0) ////��x���������ᣬ����0��
        uwReturnValue = 0;
    else if (x < 0 && y == 0) //��x���ϸ����ᣬ����180��
        uwReturnValue = 32768;
    else if (x == 0 && y == 0) //��ԭ�㣬����0��
        uwReturnValue = 0;
    return uwReturnValue;
}
#endif

void ClarkeAndPark_Convert(MATRIX_CONVERT *v)
{
    s32 temp1, temp2;
    /*Clarke */
    v->outputs.swAlpha = v->inputs.swA; //Ialpha=Ia

    temp1 = v->inputs.swB;
    temp1 = v->inputs.swA + temp1 + temp1;
    temp2 = temp1 * ONE_SQRT_3 / 32768; // ���������temp1*ONE_SQRT_3<3*2^15*0.6*2^15=1.8*2^30<2^31
    if (temp2 > 32767)
        temp2 = 32767;
    if (temp2 < -32767)
        temp2 = -32767;
    v->outputs.swBelt = temp2; // Ibelt=(2*Ib+Ia)/sqrt(3)

    /*Park*/
    temp1 = v->outputs.swAlpha * v->pTrig->hCos;
    temp2 = v->outputs.swBelt * v->pTrig->hSin;
    temp2 = (temp1 + temp2) / 32768;
    if (temp2 > 32767)
        temp2 = 32767;
    if (temp2 < -32767)
        temp2 = -32767;
    v->outputs.swDs = temp2;

    temp1 = v->outputs.swBelt * v->pTrig->hCos;
    temp2 = v->outputs.swAlpha * v->pTrig->hSin;
    temp2 = (temp1 - temp2) / 32768;
    if (temp2 > 32767)
        temp2 = 32767;
    if (temp2 < -32767)
        temp2 = -32767;
    v->outputs.swQs = temp2;
}
/*******************************************************************************
* Function Name  : ClarkeAndPark_Convert
* Description    : This function returns Ialpha /Ibelt/Id/Iq
*
* Input          : Ia/Ib  in s16 formaty
* Output         : Ialpha /Ibelt/Id/Iq in s16 format
* Return         : none.
*******************************************************************************/

void fp_abtodq(s16 ia, s16 ib, u16 u16theta, s16 *id, s16 *iq)
{
    s32 temp1, temp2;
    s16 hCos, hSin;

    hSin = Math_Sin(u16theta);
    hCos = Math_Cos(u16theta);
    /*Park*/
    temp1 = ia * hCos;
    temp2 = ib * hSin;
    temp2 = (temp1 + temp2) / 32768;
    if (temp2 > 32767)
        temp2 = 32767;
    if (temp2 < -32767)
        temp2 = -32767;
    *id = temp2;

    temp1 = ib * hCos;
    temp2 = ia * hSin;
    temp2 = (temp1 - temp2) / 32768;
    if (temp2 > 32767)
        temp2 = 32767;
    if (temp2 < -32767)
        temp2 = -32767;
    *iq = temp2;
}

void fp_dqtoab(s32 ud, s32 uq, u16 theta, s32 *ua, s32 *ub)
{
    s32 temp1, temp2;
    s16 hCos, hSin;

    hSin = Math_Sin(theta);
    hCos = Math_Cos(theta);

    temp1 = ud * hCos;
    temp2 = uq * hSin;

    temp2 = (temp1 - temp2) / 32768;
    *ua = temp2;

    temp1 = uq * hCos;
    temp2 = ud * hSin;

    temp2 = (temp1 + temp2) / 32768;
    *ub = temp2;
}
/*=====================================================================================


 Dsscription:  Inverse Park Transformation

=====================================================================================*/
void Park_Calc(s16 ua, s16 ub, s16 *pud, s16 *puq, IPARK *v)
{
    s32 temp1, temp2;
    temp1 = ua * v->pTrig->hCos;
    temp2 = ub * v->pTrig->hSin;

    *pud = ((temp1 + temp2) >> 15);
    //if(temp2>32767)temp2=32767;
    //if(temp2<-32767)temp2=-32767;

    temp1 = ub * v->pTrig->hCos;
    temp2 = ua * v->pTrig->hSin;

    *puq = ((temp1 - temp2) >> 15);
    //if(temp2>32767)temp2=32767;
    //if(temp2<-32767)temp2=-32767;
}

void InvPark_Calc(IPARK *v)
{
    s32 temp1, temp2;
    temp1 = v->inputs.slDs * v->pTrig->hCos;
    temp2 = v->inputs.slQs * v->pTrig->hSin;

    temp2 = (temp1 - temp2) / 32768;
    //if(temp2>32767)temp2=32767;
    //if(temp2<-32767)temp2=-32767;
    v->outputs.slAlpha = temp2;

    temp1 = v->inputs.slQs * v->pTrig->hCos;
    temp2 = v->inputs.slDs * v->pTrig->hSin;

    temp2 = (temp1 + temp2) / 32768;
    //if(temp2>32767)temp2=32767;
    //if(temp2<-32767)temp2=-32767;
    v->outputs.slBelt = temp2;

#if 0


// Using look-up IQ sine table
     Sine = _IQsinPU(v->Angle);
     Cosine = _IQcosPU(v->Angle);

     v->Alpha = _IQmpy(v->Ds,Cosine) - _IQmpy(v->Qs,Sine);
     v->Beta = _IQmpy(v->Qs,Cosine) + _IQmpy(v->Ds,Sine);
#endif
}
