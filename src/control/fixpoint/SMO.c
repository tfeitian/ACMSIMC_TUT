#include "smo.h"
#include "config.h"
#include "Fixpoint.h"
#include "dbglog.h"
/*
F=1-Ts*R/L
G=Ts/L
*/
u16 Atan_Functions(s16 x, s16 y); //Function in MatrixConvert.c
s16 swOmegConvCoeff;
void SMO_Intialize(SMOPOS_OBJECT *v)
{
    float tempx1;
    //   R1=RS*MAX_CURRENT/MAX_VOLTAGE;
    //  L1=LS*MAX_CURRENT/MAX_VOLTAGE;
    tempx1 = (1 - T_SAMPLE * RS / LS);          //Rs/Ls=R1/L1
    v->swFsmopos = (s16)(tempx1 * 32768 + 0.5); //+0.5ʵ����������
    tempx1 = (T_SAMPLE * MAX_VOLTAGE) / (2 * LS * MAX_CURRENT);
    //tempx1=(T_SAMPLE*MAX_VOLTAGE)/(LS*MAX_CURRENT);
    v->swGsmopos = (s16)(tempx1 * 32768 + 0.5);

    /*Please change "v->outputs.swZalpha = (long)v->swKslide*v->swIalphaError/E0; ", after change E0*/
    v->swKslide = 6000;
    //12000; //0.25*32768

    /*AccAngle=Omeg*t_speedlopp=2*PI*f*t_speedlopp=2*PI*(Pole*RPM/60)*t_speedlopp
   =Pole*RPM*(65536/60)*20/PWM_FREQUENCY=Pole*RPM*65536/(3*PWM_FREQUENCY)
  swKslf= AccAngle*5147/32768=
   */
    v->swKslfMin = (s16)((long)MIN_SPEED * MOTOR_POLES * 3431 / PWM_FREQUENCY);
    v->swKslf = v->swKslfMin;
    /*BEMF sample rate is 20 times of Speed sample rate
   Kslf=2*PI*fc/fsample, fc is cut off frequency,they are same for BEMF&speed
   Speed is nearly constant, so Speed filter reduces the speed amplitude very small
   */
    // v->swOmegKslf=v->swKslfMin*20;
    v->swOmegKslf = v->swKslfMin;
    /*
  Here speed is not in RPM. It is rotated angle(-32767-32767) in a speed loop time interval.
  The conversion coefficient is here.
   20*Tpwm*Poles*2*32768*32768/60
   */

    v->swKCoeff = (s16)(2.0 * 3.1415 * Fre_MAX * 32768.0 / PWM_FREQUENCY);
    tempx1 = (float)20 * MOTOR_POLES * 2 * 32768 / (60 * PWM_FREQUENCY);
    swOmegConvCoeff = (int)(tempx1 * 1024);
    v->swOmeg2RPM_Coeff = (long)Fre_MAX * 60 / MOTOR_POLES;
    v->swMaxSMCErr = 250;
}

/*=====================================================================================
 File name:        SMOPOS.C  (IQ version)
 Description:  Rotor Position Estimator of PMSM using Sliding-Mode Theory


=====================================================================================*/
void SMOpos_calc(SMOPOS_OBJECT *v)
{
    //s16 E0=250;//Please change "v->outputs.swZalpha = (long)v->swKslide*v->swIalphaError/E0; ", after change E0
    //   E0 = _IQ(0.5);

    // Sliding mode current observer
    v->swEstIalpha = (long)v->swFsmopos * v->swEstIalpha / 32768 + (long)v->swGsmopos * (v->inputs.swValpha - v->swEalpha - v->outputs.swZalpha) / 32768;
    v->swEstIbeta = (long)v->swFsmopos * v->swEstIbeta / 32768 + (long)v->swGsmopos * (v->inputs.swVbeta - v->swEbeta - v->outputs.swZbeta) / 32768;

    // Current errors
    v->swIalphaError = v->swEstIalpha - v->inputs.swIalpha;
    v->swIbetaError = v->swEstIbeta - v->inputs.swIbeta;

    dbglog("IalphaError", v->swIalphaError);
    dbglog("IbetaError", v->swIbetaError);
    // Sliding control calculator
    /************Alpha phase control******************************/
    if (v->swIalphaError >= v->swMaxSMCErr)
        v->outputs.swZalpha = v->swKslide;
    else if (v->swIalphaError <= -v->swMaxSMCErr)
        v->outputs.swZalpha = -v->swKslide;
    else
    {
        v->outputs.swZalpha = (long)v->swKslide * v->swIalphaError / v->swMaxSMCErr; //It may need long time to excute. needed to be optimized
                                                                                     //v->outputs.swZalpha = 48*v->swIalphaError;
    }
    /************Belt phase control******************************/
    if (v->swIbetaError >= v->swMaxSMCErr)
        v->outputs.swZbeta = v->swKslide;
    else if (v->swIbetaError <= -v->swMaxSMCErr)
        v->outputs.swZbeta = -v->swKslide;
    else
    {
        v->outputs.swZbeta = (long)v->swKslide * v->swIbetaError / v->swMaxSMCErr; //It may need long time to excute. needed to be optimized
                                                                                   //v->outputs.swZbeta =  48*v->swIbetaError;
    }

// Sliding control filter -> back EMF calculator
//����Ӧ�˲����˲�����ֹƵ����Զ���������źŻ���Ƶ�ʣ��˲�������λ�ͺ�����45��
#if 0
    Trig_Components TrigCon_test;
    extern u16  uwAngleCmd;
    TrigCon_test= Trig_Functions(uwAngleCmd);
    v->outputs.swZalpha=-TrigCon_test.hSin;
    v->outputs.swZbeta=TrigCon_test.hCos;
#endif

    // v->swKslf=(long)v->swOmegfiltered*5147/32768;//�����˲�ϵ�� Pi/20
    //if(v->swKslf < v->swKslfMin) v->swKslf = v->swKslfMin;
#if 0
    v->swOmegfiltered =v->swOmegfiltered + \
       (s16)((long)(v->swOmeg - v->swOmegfiltered)*v->swOmegKslf/32768);
#else
    v->slOmegTemp = v->swOmeg * 4;
    v->slOmegTempFiltered = v->slOmegTempFiltered + (v->slOmegTemp - v->slOmegTempFiltered) * v->swOmegKslf / 32768;
    v->swOmegfiltered = v->slOmegTempFiltered / 4;
#endif

    v->swKslf = (long)v->swOmegfiltered * v->swKCoeff / 32768; //�����˲�ϵ�� 3*Pi*32768/100
    if (v->swKslf < v->swKslfMin)
        v->swKslf = v->swKslfMin;

    v->swEalpha = v->swEalpha + (s16)((long)v->swKslf * (v->outputs.swZalpha - v->swEalpha) / 32768);
    v->swEbeta = v->swEbeta + (s16)((long)v->swKslf * (v->outputs.swZbeta - v->swEbeta) / 32768);

    v->outputs.swEalphaFiltered = v->outputs.swEalphaFiltered + (s16)((long)v->swKslf * (v->swEalpha - v->outputs.swEalphaFiltered) / 32768);
    v->outputs.swEbetaFiltered = v->outputs.swEbetaFiltered + (s16)((long)v->swKslf * (v->swEbeta - v->outputs.swEbetaFiltered) / 32768);

    dbglog("EalphaFiltered", v->outputs.swEalphaFiltered);
    dbglog("EbetaFiltered", v->outputs.swEbetaFiltered); // Rotor angle calculator -> Theta = atan(-Ealpha,Ebeta)
// Format of _IQ funtion   = _IQatan2PU(sin,cos)
// v->outputs.uwTheta = _IQtoIQ16(_IQatan2PU(_IQ15toIQ(-v->swEalpha),_IQ15toIQ(v->swEbeta)));
//Ealpha=-w*Flux*sin()    Ebelt=w*Flux*cos()
//   v->swEalphaFiltered =v->outputs.swZalpha;
//   v->swEbetaFiltered= v->outputs.swZbeta;
// v->outputs.swTheta=Atan_Functions(v->swEbeta,-v->swEalpha);
#if 0
    v->outputs.swTheta=Atan_Functions(v->outputs.swEbetaFiltered,-v->outputs.swEalphaFiltered)+DEGREE90;
    v->outputs.swAccTheta += v->outputs.swTheta - v->outputs.swPreTheta;
    v->outputs.swPreTheta=v->outputs.swTheta;

    v->ubAccumThetaCnt++;
    if (v->ubAccumThetaCnt >= 20)//Notice, this 20 means the speed loop excute frequency is 20 times of PWM frequency
     {
      // v->swOmeg = v->outputs.swAccTheta;
       v->ubAccumThetaCnt = 0;
       v->outputs.swAccTheta = 0;
     //  v->isTimeToRun=1;
       ClosedLoopCommand.ubIstimeToRunSpeedLoop=1;
     }
#endif
    v->ubAccumThetaCnt++;
    if (v->ubAccumThetaCnt >= 20) //Notice, this 20 means the speed loop excute frequency is 20 times of PWM frequency
    {
        // v->swOmeg = v->outputs.swAccTheta;
        v->ubAccumThetaCnt = 0;
        //  v->outputs.swAccTheta = 0;
        //  v->isTimeToRun=1;
        // ClosedLoopCommand.ubIstimeToRunSpeedLoop = 1;
    }
}
