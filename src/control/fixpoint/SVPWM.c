/*=====================================================================================
 File name:        SVGEN_AB.C  (IQ version)

 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  Space-vector PWM generation based on Alpha-Belt components

=====================================================================================
 History:
-------------------------------------------------------------------------------------
 04-15-2005	Version 3.20
-------------------------------------------------------------------------------------*/
#include "userdefine.h"
#include "Svpwm.h"

u16 uwDuty[2]; //uwDuty[0]���ڴ�������ʸ��ʱ�䣬uwDuty[1]���ڴ�ŵڶ���ʸ��ʱ��
u8 ubBigCurrentNo = 0;
u8 ubBigCurrentNoOld = 0;

#define SVPWM_MODE SVPWM_SEVEN_SEGMENTS

void Svpwm_Alpha_Belt_Initialize(SVGENAB *v)
{
    v->Ualpha = 0;
    v->Ubeta = 0;
    v->Ta = 0;
    v->Tb = 0;
    v->Tc = 0;
    v->ubSector = 0;
}
/*=====================================================================================
 File name:        SVGEN_AB.C  (IQ version)

 Originator:	Digital Control Systems Group
			Texas Instruments

 Description:  Space-vector PWM generation based on Alpha-Belt components

=====================================================================================
Ualpha Ubeta should be scaled.
here  Vdc/squt(3)   is equal  32768   _IQ(1)
Ualpha Ubeta should less than _IQ(1.15)    _IQ(2/sqrt(3))
-------------------------------------------------------------------------------------*/
void Svpwm_Alpha_Belt_Calc(SVGENAB *v)
{
    s32 Va, Vb, Vc, t1, t2;
    v->ubSector = 0;

    // Inverse clarke transformation
    Va = v->Ubeta;
    Vb = -(v->Ubeta / 2) + (v->Ualpha * HALF_SQRT_3) / 32768; // 0.8660254 = sqrt(3)/2
    Vc = -(v->Ubeta / 2) - (v->Ualpha * HALF_SQRT_3) / 32768; // 0.8660254 = sqrt(3)/2

    // 60 degree v->ubSector determination
    if (Va > 0)
        v->ubSector = 1;
    if (Vb > 0)
        v->ubSector = v->ubSector + 2;
    if (Vc > 0)
        v->ubSector = v->ubSector + 4;

    // X,Y,Z (Va,Vb,Vc) calculations
    Va = v->Ubeta;                                           // X = Va
    Vb = (v->Ubeta / 2) + (v->Ualpha * HALF_SQRT_3) / 32768; // Y = Vb
    Vc = (v->Ubeta / 2) - (v->Ualpha * HALF_SQRT_3) / 32768; // Z = Vc

#if SVPWM_MODE == SVPWM_SEVEN_SEGMENTS
    if (v->ubSector == 0) // v->ubSector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    {
        v->Ta = 32767;
        v->Tb = 32767;
        v->Tc = 32767;
    }
    if (v->ubSector == 1) // v->ubSector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)
    {
        //010,110
        t1 = Vc;
        t2 = Vb;
        if (T_PWM > t1 + t2)
            v->Tb = (T_PWM - t1 - t2) / 2; // tbon = (1-t1-t2)/2
        else
            v->Tb = 0;
        v->Ta = v->Tb + t1; // taon = tbon+t1
        v->Tc = v->Ta + t2; // tcon = taon+t2
        uwDuty[0] = v->Tb;
        //Ib& -Ic
    }
    else if (v->ubSector == 2) // v->ubSector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)
    {                          //100,101
        t1 = Vb;
        t2 = -Va;
        if (T_PWM > t1 + t2)
            v->Ta = (T_PWM - t1 - t2) / 2; // taon = (1-t1-t2)/2
        else
            v->Ta = 0;
        v->Tc = v->Ta + t1; // tcon = taon+t1
        v->Tb = v->Tc + t2; // tbon = tcon+t2
        uwDuty[0] = v->Ta;
        //Ia& -Ib
    }
    else if (v->ubSector == 3) // v->ubSector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)
    {
        //100,110
        t1 = -Vc;
        t2 = Va;
        if (T_PWM > t1 + t2)
            v->Ta = (T_PWM - t1 - t2) / 2; // taon = (1-t1-t2)/2
        else
            v->Ta = 0;
        v->Tb = v->Ta + t1; // tbon = taon+t1
        v->Tc = v->Tb + t2; // tcon = tbon+t2
        uwDuty[0] = v->Ta;
        //Ia& -Ic
    }
    else if (v->ubSector == 4) // v->ubSector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)
    {                          //001,011
        t1 = -Va;
        t2 = Vc;
        if (T_PWM > t1 + t2)
            v->Tc = (T_PWM - t1 - t2) / 2; // tcon = (1-t1-t2)/2
        else
            v->Tc = 0;
        v->Tb = v->Tc + t1; // tbon = tcon+t1
        v->Ta = v->Tb + t2; // taon = tbon+t2
        uwDuty[0] = v->Tc;
        //Ic& -Ia
    }
    else if (v->ubSector == 5) // v->ubSector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)
    {                          //010,011
        t1 = Va;
        t2 = -Vb;
        if (T_PWM > t1 + t2)
            v->Tb = (T_PWM - t1 - t2) / 2; // tbon = (1-t1-t2)/2
        else
            v->Tb = 0;
        v->Tc = v->Tb + t1; // tcon = tbon+t1
        v->Ta = v->Tc + t2; // taon = tcon+t2
        uwDuty[0] = v->Tb;
        //Ib& -Ia
    }
    else if (v->ubSector == 6) // v->ubSector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)
    {                          //001,101
        t1 = -Vb;
        t2 = -Vc;
        if (T_PWM > t1 + t2)
            v->Tc = (T_PWM - t1 - t2) / 2; // tcon = (1-t1-t2)/2
        else
            v->Tc = 0;
        v->Ta = v->Tc + t1; // taon = tcon+t1
        v->Tb = v->Ta + t2; // tbon = taon+t2
        uwDuty[0] = v->Tc;
        //Ic& -Ib
    }
#endif

#if SVPWM_MODE == SVPWM_FIVE_SEGMENTS
    /*
   ��ѹʸ������5��ʽ����ʸ��ֻ��000

    */
    if (v->ubSector == 0) // v->ubSector 0: this is special case for (Ualpha,Ubeta) = (0,0)
    {
        v->Ta = 32767;
        v->Tb = 32767;
        v->Tc = 32767;
    }
    if (v->ubSector == 1) // v->ubSector 1: t1=Z and t2=Y (abc ---> Tb,Ta,Tc)
    {
        t1 = Vc;
        t2 = Vb;
        Va = T_PWM - t1 - t2;
        if (Va > 0)
            v->Tb = (T_PWM - t1 - t2); // tbon = (1-t1-t2)
        else
            v->Tb = 0;
        v->Ta = v->Tb + t1; // taon = tbon+t1
        v->Tc = 32767;      // tcon = taon+t2
        uwDuty[0] = v->Tb;
        /*����C���A�����*/
    }
    else if (v->ubSector == 2) // v->ubSector 2: t1=Y and t2=-X (abc ---> Ta,Tc,Tb)
    {
        t1 = Vb;
        t2 = -Va;
        Va = T_PWM - t1 - t2;
        if (Va > 0)
            v->Ta = T_PWM - t1 - t2; // taon = (1-t1-t2)/2
        else
            v->Ta = 0;
        v->Tc = v->Ta + t1; // tcon = taon+t1
        v->Tb = 32767;      // tbon = tcon+t2
        uwDuty[0] = v->Ta;

        /*����B���C�����*/
    }
    else if (v->ubSector == 3) // v->ubSector 3: t1=-Z and t2=X (abc ---> Ta,Tb,Tc)
    {
        t1 = -Vc;
        t2 = Va;
        Va = T_PWM - t1 - t2;
        if (Va > 0)
            v->Ta = T_PWM - t1 - t2; // taon = (1-t1-t2)
        else
            v->Ta = 0;
        v->Tb = v->Ta + t1; // tbon = taon+t1
        v->Tc = 32767;      // tcon = tbon+t2
        uwDuty[0] = v->Ta;
        /*����C���B�����*/
    }
    else if (v->ubSector == 4) // v->ubSector 4: t1=-X and t2=Z (abc ---> Tc,Tb,Ta)
    {
        t1 = -Va;
        t2 = Vc;
        Va = T_PWM - t1 - t2;
        if (Va > 0)
            v->Tc = T_PWM - t1 - t2; // tcon = (1-t1-t2)/2
        else
            v->Tc = 0;
        v->Tb = v->Tc + t1; // tbon = tcon+t1
        v->Ta = 32767;      // taon = tbon+t2
        uwDuty[0] = v->Tc;
    }
    else if (v->ubSector == 5) // v->ubSector 5: t1=X and t2=-Y (abc ---> Tb,Tc,Ta)
    {
        t1 = Va;
        t2 = -Vb;
        Va = T_PWM - t1 - t2;
        if (Va > 0)
            v->Tb = T_PWM - t1 - t2; // tbon = (1-t1-t2)/2
        else
            v->Tb = 0;
        v->Tc = v->Tb + t1; // tcon = tbon+t1
        v->Ta = 32767;      // taon = tcon+t2
        uwDuty[0] = v->Tb;
    }
    else if (v->ubSector == 6) // v->ubSector 6: t1=-Y and t2=-Z (abc ---> Tc,Ta,Tb)
    {
        t1 = -Vb;
        t2 = -Vc;
        Va = T_PWM - t1 - t2;
        if (Va > 0)
            v->Tc = T_PWM - t1 - t2; // tcon = (1-t1-t2)/2
        else
            v->Tc = 0;
        v->Ta = v->Tc + t1; // taon = tcon+t1
        v->Tb = 32767;
        uwDuty[0] = v->Tc;
    }
#endif
    // if (uwDuty[0] < MIN_CURRENT_SMAPLE_INTERVAL)
    // NO_ENOUGH_TIME2CURRENT_SAMPLE;
}
