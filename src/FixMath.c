#include "FixMath.h"

s16 const wSin_Table[361] = {
    0,
    143,
    286,
    429,
    572,
    715,
    858,
    1001,
    1144,
    1286,
    1429,
    1572,
    1715,
    1858,
    2000,
    2143,
    2286,
    2428,
    2571,
    2713,
    2856,
    2998,
    3141,
    3283,
    3425,
    3567,
    3709,
    3851,
    3993,
    4135,
    4277,
    4419,
    4560,
    4702,
    4843,
    4985,
    5126,
    5267,
    5408,
    5549,
    5690,
    5831,
    5971,
    6112,
    6252,
    6393,
    6533,
    6673,
    6813,
    6952,
    7092,
    7232,
    7371,
    7510,
    7649,
    7788,
    7927,
    8066,
    8204,
    8343,
    8481,
    8619,
    8757,
    8894,
    9032,
    9169,
    9306,
    9443,
    9580,
    9717,
    9853,
    9989,
    10126,
    10261,
    10397,
    10533,
    10668,
    10803,
    10938,
    11073,
    11207,
    11341,
    11475,
    11609,
    11743,
    11876,
    12009,
    12142,
    12275,
    12407,
    12539,
    12671,
    12803,
    12935,
    13066,
    13197,
    13328,
    13458,
    13588,
    13718,
    13848,
    13977,
    14107,
    14235,
    14364,
    14492,
    14621,
    14748,
    14876,
    15003,
    15130,
    15257,
    15383,
    15509,
    15635,
    15761,
    15886,
    16011,
    16135,
    16260,
    16383,
    16507,
    16630,
    16754,
    16876,
    16999,
    17121,
    17242,
    17364,
    17485,
    17606,
    17726,
    17846,
    17966,
    18085,
    18204,
    18323,
    18441,
    18559,
    18677,
    18794,
    18911,
    19028,
    19144,
    19260,
    19375,
    19491,
    19605,
    19720,
    19834,
    19947,
    20061,
    20173,
    20286,
    20398,
    20510,
    20621,
    20732,
    20842,
    20952,
    21062,
    21172,
    21280,
    21389,
    21497,
    21605,
    21712,
    21819,
    21925,
    22031,
    22137,
    22242,
    22347,
    22451,
    22555,
    22659,
    22762,
    22864,
    22967,
    23068,
    23170,
    23271,
    23371,
    23471,
    23571,
    23670,
    23768,
    23867,
    23964,
    24062,
    24158,
    24255,
    24351,
    24446,
    24541,
    24636,
    24730,
    24823,
    24916,
    25009,
    25101,
    25193,
    25284,
    25375,
    25465,
    25554,
    25644,
    25732,
    25821,
    25909,
    25996,
    26083,
    26169,
    26255,
    26340,
    26425,
    26509,
    26593,
    26676,
    26759,
    26841,
    26923,
    27004,
    27085,
    27165,
    27245,
    27324,
    27403,
    27481,
    27558,
    27635,
    27712,
    27788,
    27863,
    27938,
    28013,
    28087,
    28160,
    28233,
    28305,
    28377,
    28448,
    28519,
    28589,
    28659,
    28728,
    28796,
    28864,
    28932,
    28998,
    29065,
    29130,
    29196,
    29260,
    29324,
    29388,
    29451,
    29513,
    29575,
    29636,
    29697,
    29757,
    29817,
    29876,
    29934,
    29992,
    30049,
    30106,
    30162,
    30218,
    30273,
    30327,
    30381,
    30434,
    30487,
    30539,
    30591,
    30642,
    30692,
    30742,
    30791,
    30840,
    30888,
    30935,
    30982,
    31028,
    31074,
    31119,
    31163,
    31207,
    31250,
    31293,
    31335,
    31377,
    31418,
    31458,
    31498,
    31537,
    31575,
    31613,
    31650,
    31687,
    31723,
    31759,
    31794,
    31828,
    31862,
    31895,
    31927,
    31959,
    31990,
    32021,
    32051,
    32080,
    32109,
    32137,
    32165,
    32192,
    32218,
    32244,
    32269,
    32294,
    32318,
    32341,
    32364,
    32386,
    32407,
    32428,
    32448,
    32468,
    32487,
    32505,
    32523,
    32540,
    32556,
    32572,
    32587,
    32602,
    32616,
    32630,
    32642,
    32654,
    32666,
    32677,
    32687,
    32697,
    32706,
    32714,
    32722,
    32729,
    32736,
    32742,
    32747,
    32752,
    32756,
    32759,
    32762,
    32764,
    32766,
    32767,
    32767};
#if 1
u16 const uwActan_Table[513] = {
    0,
    20,
    41,
    61,
    81,
    102,
    122,
    143,
    163,
    183,
    204,
    224,
    244,
    265,
    285,
    305,
    326,
    346,
    367,
    387,
    407,
    428,
    448,
    468,
    489,
    509,
    529,
    550,
    570,
    590,
    610,
    631,
    651,
    671,
    692,
    712,
    732,
    752,
    773,
    793,
    813,
    833,
    854,
    874,
    894,
    914,
    935,
    955,
    975,
    995,
    1015,
    1036,
    1056,
    1076,
    1096,
    1116,
    1136,
    1156,
    1177,
    1197,
    1217,
    1237,
    1257,
    1277,
    1297,
    1317,
    1337,
    1357,
    1377,
    1397,
    1417,
    1437,
    1457,
    1477,
    1497,
    1517,
    1537,
    1557,
    1577,
    1597,
    1617,
    1637,
    1656,
    1676,
    1696,
    1716,
    1736,
    1756,
    1775,
    1795,
    1815,
    1835,
    1854,
    1874,
    1894,
    1914,
    1933,
    1953,
    1973,
    1992,
    2012,
    2031,
    2051,
    2071,
    2090,
    2110,
    2129,
    2149,
    2168,
    2188,
    2207,
    2227,
    2246,
    2266,
    2285,
    2305,
    2324,
    2343,
    2363,
    2382,
    2401,
    2421,
    2440,
    2459,
    2478,
    2498,
    2517,
    2536,
    2555,
    2574,
    2594,
    2613,
    2632,
    2651,
    2670,
    2689,
    2708,
    2727,
    2746,
    2765,
    2784,
    2803,
    2822,
    2841,
    2860,
    2879,
    2897,
    2916,
    2935,
    2954,
    2973,
    2991,
    3010,
    3029,
    3047,
    3066,
    3085,
    3103,
    3122,
    3141,
    3159,
    3178,
    3196,
    3215,
    3233,
    3252,
    3270,
    3289,
    3307,
    3325,
    3344,
    3362,
    3380,
    3399,
    3417,
    3435,
    3453,
    3472,
    3490,
    3508,
    3526,
    3544,
    3562,
    3580,
    3599,
    3617,
    3635,
    3653,
    3670,
    3688,
    3706,
    3724,
    3742,
    3760,
    3778,
    3796,
    3813,
    3831,
    3849,
    3867,
    3884,
    3902,
    3920,
    3937,
    3955,
    3972,
    3990,
    4007,
    4025,
    4042,
    4060,
    4077,
    4095,
    4112,
    4129,
    4147,
    4164,
    4181,
    4199,
    4216,
    4233,
    4250,
    4267,
    4284,
    4302,
    4319,
    4336,
    4353,
    4370,
    4387,
    4404,
    4421,
    4438,
    4454,
    4471,
    4488,
    4505,
    4522,
    4539,
    4555,
    4572,
    4589,
    4605,
    4622,
    4639,
    4655,
    4672,
    4688,
    4705,
    4721,
    4738,
    4754,
    4771,
    4787,
    4803,
    4820,
    4836,
    4852,
    4869,
    4885,
    4901,
    4917,
    4933,
    4949,
    4966,
    4982,
    4998,
    5014,
    5030,
    5046,
    5062,
    5078,
    5094,
    5109,
    5125,
    5141,
    5157,
    5173,
    5188,
    5204,
    5220,
    5235,
    5251,
    5267,
    5282,
    5298,
    5313,
    5329,
    5344,
    5360,
    5375,
    5391,
    5406,
    5421,
    5437,
    5452,
    5467,
    5483,
    5498,
    5513,
    5528,
    5543,
    5559,
    5574,
    5589,
    5604,
    5619,
    5634,
    5649,
    5664,
    5679,
    5694,
    5708,
    5723,
    5738,
    5753,
    5768,
    5782,
    5797,
    5812,
    5826,
    5841,
    5856,
    5870,
    5885,
    5899,
    5914,
    5928,
    5943,
    5957,
    5972,
    5986,
    6000,
    6015,
    6029,
    6043,
    6058,
    6072,
    6086,
    6100,
    6114,
    6128,
    6142,
    6157,
    6171,
    6185,
    6199,
    6213,
    6227,
    6240,
    6254,
    6268,
    6282,
    6296,
    6310,
    6323,
    6337,
    6351,
    6365,
    6378,
    6392,
    6406,
    6419,
    6433,
    6446,
    6460,
    6473,
    6487,
    6500,
    6514,
    6527,
    6540,
    6554,
    6567,
    6580,
    6594,
    6607,
    6620,
    6633,
    6646,
    6660,
    6673,
    6686,
    6699,
    6712,
    6725,
    6738,
    6751,
    6764,
    6777,
    6790,
    6803,
    6815,
    6828,
    6841,
    6854,
    6867,
    6879,
    6892,
    6905,
    6917,
    6930,
    6943,
    6955,
    6968,
    6980,
    6993,
    7005,
    7018,
    7030,
    7043,
    7055,
    7068,
    7080,
    7092,
    7105,
    7117,
    7129,
    7141,
    7154,
    7166,
    7178,
    7190,
    7202,
    7214,
    7226,
    7238,
    7250,
    7262,
    7274,
    7286,
    7298,
    7310,
    7322,
    7334,
    7346,
    7358,
    7369,
    7381,
    7393,
    7405,
    7416,
    7428,
    7440,
    7451,
    7463,
    7475,
    7486,
    7498,
    7509,
    7521,
    7532,
    7544,
    7555,
    7567,
    7578,
    7589,
    7601,
    7612,
    7623,
    7635,
    7646,
    7657,
    7668,
    7679,
    7691,
    7702,
    7713,
    7724,
    7735,
    7746,
    7757,
    7768,
    7779,
    7790,
    7801,
    7812,
    7823,
    7834,
    7845,
    7856,
    7866,
    7877,
    7888,
    7899,
    7910,
    7920,
    7931,
    7942,
    7952,
    7963,
    7974,
    7984,
    7995,
    8005,
    8016,
    8026,
    8037,
    8047,
    8058,
    8068,
    8079,
    8089,
    8100,
    8110,
    8120,
    8131,
    8141,
    8151,
    8161,
    8172,
    8182,
    8192,

};
#endif

void Trig_Functions(Trig_Components *p)
{
    u16 hindex = 0;
    if (p->uwAngle < DEGREE90)
    {
        hindex = ((u32)p->uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = wSin_Table[hindex];
        p->hCos = wSin_Table[360 - hindex];
    }
    else if (p->uwAngle < DEGREE180)
    {
        p->uwAngle = DEGREE180 - p->uwAngle;
        hindex = ((u32)p->uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = wSin_Table[hindex];
        ;
        p->hCos = -wSin_Table[360 - hindex];
    }
    else if (p->uwAngle < DEGREE270)
    {
        p->uwAngle = p->uwAngle - DEGREE180;
        hindex = ((u32)p->uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = -wSin_Table[hindex];
        p->hCos = -wSin_Table[360 - hindex];
    }
    else if (p->uwAngle < DEGREE360)
    {
        p->uwAngle = DEGREE360 - p->uwAngle;
        hindex = ((u32)p->uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        p->hSin = -wSin_Table[hindex];
        p->hCos = wSin_Table[360 - hindex];
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
        hindex = ((u32)uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = wSin_Table[hindex];
        Local_Components.hCos = wSin_Table[360 - hindex];
    }
    else if (uwAngle < DEGREE180)
    {
        uwAngle = DEGREE180 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = wSin_Table[hindex];
        Local_Components.hCos = -wSin_Table[360 - hindex];
    }
    else if (uwAngle < DEGREE270)
    {
        uwAngle = uwAngle - DEGREE180;
        hindex = ((u32)uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = -wSin_Table[hindex];
        Local_Components.hCos = -wSin_Table[360 - hindex];
    }
    else if (uwAngle < DEGREE360)
    {
        uwAngle = DEGREE360 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> 11; // sin����Ϊÿ��0.25����һ����Ӧ����ֵ,hindex=uwAngle*360/(65536*0.25)
        Local_Components.hSin = -wSin_Table[hindex];
        Local_Components.hCos = wSin_Table[360 - hindex];
    }
    return (Local_Components);
}
#endif

s16 Math_Sin(u16 uwAngle)
{
    u16 hindex = 0;

    if (uwAngle < DEGREE90)
    {
        hindex = ((u32)uwAngle * 45) >> 11;
        return wSin_Table[hindex];
    }
    else if (uwAngle < DEGREE180)
    {
        uwAngle = DEGREE180 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> 11;
        return wSin_Table[hindex];
    }
    else if (uwAngle < DEGREE270)
    {
        uwAngle = uwAngle - DEGREE180;
        hindex = ((u32)uwAngle * 45) >> 11;
        return -wSin_Table[hindex];
    }
    else if (uwAngle < DEGREE360)
    {
        uwAngle = DEGREE360 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> 11;
        return -wSin_Table[hindex];
    }
    return 0;
}

s16 Math_Cos(u16 uwAngle)
{
    u16 hindex = 0;

    if (uwAngle < DEGREE90)
    {
        hindex = ((u32)uwAngle * 45) >> 11;
        return wSin_Table[360 - hindex];
    }
    else if (uwAngle < DEGREE180)
    {
        uwAngle = DEGREE180 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> 11;
        return -wSin_Table[360 - hindex];
    }
    else if (uwAngle < DEGREE270)
    {
        uwAngle = uwAngle - DEGREE180;
        hindex = ((u32)uwAngle * 45) >> 11;
        return -wSin_Table[360 - hindex];
    }
    else if (uwAngle < DEGREE360)
    {
        uwAngle = DEGREE360 - uwAngle;
        hindex = ((u32)uwAngle * 45) >> 11;
        return wSin_Table[360 - hindex];
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
