#include "FixMath.h"

#define SIN_TABLE_LENGTH 721
#define SIN_CAL_INDEX (SIN_TABLE_LENGTH - 1)
#define SIN_TABLE_SHIFT 10

#if 1

#endif

void Trig_Functions(Trig_Components *p)
{
    u16 hindex = 0;
    u16 uwAngleTmp;
    if (p->uwAngle < DEGREE90)
    {
        hindex = ((u32)p->uwAngle * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = wSin_Table[hindex];
        p->hCos = wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (p->uwAngle < DEGREE180)
    {
        uwAngleTmp = DEGREE180 - p->uwAngle;
        hindex = ((u32)uwAngleTmp * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = wSin_Table[hindex];
        p->hCos = -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else if (p->uwAngle < DEGREE270)
    {
        uwAngleTmp = p->uwAngle - DEGREE180;
        hindex = ((u32)uwAngleTmp * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = -wSin_Table[hindex];
        p->hCos = -wSin_Table[SIN_CAL_INDEX - hindex];
    }
    else
    {
        uwAngleTmp = DEGREE360 - p->uwAngle;
        hindex = ((u32)uwAngleTmp * 45) >> SIN_TABLE_SHIFT; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
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
/**
  * @brief  Low pass filter.
  * @param  Input s16Coef coef*(2**15)
  * @retval int16_t Filted value
  */
s16 s16LP_Filter(s16 s16Input, s16 s16Coef, s32 *s32OldInput)
{
    s32 s32filted = (s32)s16Input * s16Coef + ((s64)(32768 - s16Coef) * (*s32OldInput) >> 15);
    *s32OldInput = (s32filted);
    return (s16)(s32filted >> 15);
}
/**
  * @brief  It calculates the square root of a non-negative s32. It returns 0
  *         for negative s32.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t Math_Sqrt(int32_t wInput)
{
    uint8_t biter = 0u;
    int32_t wtemproot;
    int32_t wtemprootnew;

    if (wInput > 0)
    {

        if (wInput <= (int32_t)2097152)
        {
            wtemproot = (int32_t)128;
        }
        else
        {
            wtemproot = (int32_t)8192;
        }

        do
        {
            wtemprootnew = (wtemproot + wInput / wtemproot) / (int32_t)2;
            if (wtemprootnew == wtemproot)
            {
                biter = 6u;
            }
            else
            {
                biter++;
                wtemproot = wtemprootnew;
            }
        } while (biter < 6u);
    }
    else
    {
        wtemprootnew = (int32_t)0;
    }

    return (wtemprootnew);
}

u32 iatan(s32 arg)
{
    u16 arg16;
    bool sign_arg;
    u32 tmpU32;
    u32 atanInt;
    u8 addr;
    u16 rest;
    u16 low_word;

    /* check for negative argument */
    if (arg < 0)
    {
        arg = -arg; // argument has to be positive always
        sign_arg = TRUE;
    }
    else
    {
        sign_arg = FALSE;
    }
    arg16 = (u16)(arg >> 16);
    /* Check if the argument is below 8*2^16*/
    if (arg16 < 8)
    {
        low_word = (u16)(arg >> 3);
        addr = (u8)(low_word >> 8); // take first 8 bits (addr)
        rest = low_word & 0x00FF;   // take last 8 bits (rest)
        atanInt = ATAN_TAB1[addr];
        tmpU32 = rest * (u32)(ATAN_TAB1[addr + 1] - (u16)atanInt);
        atanInt = ((atanInt << 8) + tmpU32) << 6;
    }
    else if (arg16 < 128)
    {
        low_word = (u16)(arg >> 7);
        addr = (u8)(low_word >> 8); // take first 8 bits (addr)
        rest = low_word & 0x00FF;   // take last 8 bits (rest)
        atanInt = ATAN_TAB2[addr];
        tmpU32 = rest * (u32)(ATAN_TAB2[addr + 1] - (u16)atanInt);
        atanInt = ((atanInt << 8) + tmpU32) << 6;
    }
    else
    {
        atanInt = 1068391149 + 162 * (u32)arg16;
    }

    /* subtract from 2*pi, if negative argument */
    if (sign_arg)
        atanInt = (u32)((-1) * (s32)atanInt);

    return (atanInt);
}
#define S32_MAX ((s32)2147483647)
#define S32_MIN ((s32)(-2147483647 - 1))

u32 U32AbsS32(s32 i)
{
    if (i >= 0)
        return i;
    else
        return -i;
}

u16 iatan2(s32 lY, s32 lX)
{
    u16 dwAngle;
    s32 lTemp;

    if (lX == 0 && lY == 0)
    {
        return 0;
    }

    if ((lX == 0) || (U32AbsS32(lX) < 256))
    {
        if (lY >= 0)
        {
            return 16384; // PI/2
        }
        else
        {
            return 49152; // -PI/2
        }
    }
    lTemp = lY / (lX >> 8);

    // y = LIMIT(llTemp,S32_MAX,S32_MIN);
    if (lTemp > (0x7FFFFFFF >> 8))
    {
        lY = S32_MAX;
    }
    else if (lTemp < (-(0x7FFFFFFF >> 8)))
    {
        lY = S32_MIN;
    }
    else
    {
        lY = (s32)(lTemp << 8);
    }
    dwAngle = (u16)(iatan(lY) >> 16);
    if (lX > 0)
    {
        return dwAngle;
    }
    else
    {
        return dwAngle + 32768; // atan() + PI
    }
}