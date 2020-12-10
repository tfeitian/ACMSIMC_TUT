#include "controller.h"
#include "motor.h"
#include "tools.h"
#include "VFControl.h"
#include "math.h"
#include "Fixpoint.h"
#include "Fixmath.h"
#include "svpwm.h"
#include "PowerModuleCTL.h"
#include "ramp.h"
#include "smo.h"
#include "dbglog.h"
#include "PI_Adjuster.h"

static float power_p, dw_p;
u16 theta;
static int vc_count = 0;
s16 wset = 0;
struct PI_Reg pi_Phi;
PIDREG_OBJECT QRegulate;

void vffix_control(double speed_cmd, double speed_cmd_dot)
{
    s16 dw = 0;
    s16 wref = FP_SPEED2RAD(speed_cmd);

    /*     if (speed_cmd >= 200)
    {
        dw = 0;
    } */

    float ia = ACM.ial;
    float ib = ACM.ibe;
    float K = 0;

    float power = 3 / 2 * (CTRL.ual * ia + CTRL.ube * ib);
    float HFP = HighPassFilter_RC_1order(&power, &power_p, &dw_p, 16000);

    if (wset != 0)
    {
        dw = -K / ((float)wref * Freq_MAX / 32768) * HFP;
    }
    dbg_tst(23, dw);
    // dw = 0;
    wset = wref + FP_RAD(dw);

    s16 vcomp = FP_VOLTAGE(10.0 / 200.0 * speed_cmd);
    if (vcomp > FP_VOLTAGE(10.0))
    {
        vcomp = FP_VOLTAGE(10.0);
    }
    s32 vout = FP_VOLTAGE((float)wset * Freq_MAX * ACM.KE / 32768) + vcomp;

    theta += FP_THETA((float)wset * Freq_MAX / 32768 * TS);

    if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;
        CTRL.ual = FLOAT_V(vout * Math_Cos(theta) >> 15);
        CTRL.ube = FLOAT_V(vout * Math_Sin(theta) >> 15);
    }

    dbg_tst(17, vout);

    dbg_tst(24, wset);
    dbg_tst(25, wref);
    dbg_tst(15, theta);
    dbg_tst(14, FP_THETA(ACM.theta_d));
    dbg_tst(21, vcomp);
    dbg_tst(11, power);
    dbg_tst(12, HFP);
}

float fke = 0.405;

#define RAMP_STEP (1024 / 5 * 3)

u16 u16RampStep = 0;

s16 wref = 0;

s32 wsetold = 0;

#define DC_BRAKE_TIME 0.0 * 16000

u32 u32DcRunTime = 0;

extern SVGENAB sv1;

bool bsmoInit = false;

static u16 transfertime = 0;

float ftheta = 0;

double istotalold, iscosold, qlold, dwold, powerold;

u16 u16Rs, u16Ke;

float fku = 141.89;
float fki = 1638.4;
float fkr = 88.681;
float fkphi = 5572;
float fkw = 26.08;
float fkp = 113.5;

s32 s16IsTotalOld, s16IsCosphiOld, s16pOld, s32qlold, s32dwold, s32Wv;
void ufinit(void)
{
    wsetold = 0;
    u32DcRunTime = 0;

    pi_Phi.Kp = 0;
    //0.05;
    //0.5;
    pi_Phi.Ti = 0.02;
    //ACM.Lq / ACM.R;
    pi_Phi.Ki = 0.00005;
    //pi_Phi.Kp / pi_Phi.Ti;
    pi_Phi.i_state = 0;
    pi_Phi.i_limit = 10000;

    u16Rs = (u16)(ACM.R * fkr);
    u16Ke = (u16)(ACM.KE * fkphi);

    PidReg_Intialize(&QRegulate);
    QRegulate.inputs.Kp = 0;
    QRegulate.inputs.Ki = 1;

    s16IsTotalOld = 0;
    s16IsCosphiOld = 0;
    s16pOld = 0;
    s32qlold = 0;
    s32dwold = 0;
}

float uf_control(double speedref, double noused)
{
    float wref = speedref * RPM_2_RAD_PER_SEC(MOTOR_POLES);
    float v1ref = ACM.KE * wref;
    float v0 = 0;
    float dv = 0;

    float ua, ub, ia, ib;
    float K = 20;
    ua = CTRL.ual;
    ub = CTRL.ube;

    ia = ACM.ial;
    ib = ACM.ibe;

    float istotal = sqrt(ia * ia + ib * ib);
    float is_theta = theta_round(atan2(ib, ia));
    float thetaPhi = theta_round(ftheta - is_theta);

    float iscosphi = istotal * cos(thetaPhi);

    // ufcontrol0(speedref, noused);

    dbglog("uffix-is", istotal);
    dbglog("uffix-iscosphi", iscosphi);
    dbglog("uffix-is_theta", is_theta);
    dbglog("uffix-thetaPhi", thetaPhi);

    float p = 3 / 2 * (ua * ia + ub * ib);
    float q = 3 / 2 * (ub * ia - ua * ib);
    float hfp0 = p - LP_Filter(p, 0.0001, &powerold);
    float hfp = HighPassFilter_RC_1order(&p, &power_p, &dw_p, 16000);
    hfp = hfp0;
    float phi = atan2f(q, p);
    float dw = 0;

    if (wref != 0)
    {
        dw = K / wref * hfp;
    }
    // dw = 0;
    dw = LP_Filter(dw, 0.01, &dwold);

    //Speed compensation
    float wv = wref - dw;

    if (wv < 0)
    {
        wv = 0;
    }

    ftheta += wv / 16000;
    ftheta = theta_round(ftheta);

    float isfilted = LP_Filter(istotal, 0.01, &istotalold);
    dbglog("uffix-isfilted", isfilted);
    float iscosfilted = LP_Filter(iscosphi, 0.01, &iscosold);
    float ftemp = wv * ACM.KE * wv * ACM.KE + ACM.R * iscosfilted * ACM.R * iscosfilted - ACM.R * isfilted * ACM.R * isfilted;

    float us = iscosfilted * ACM.R;
    dbglog("uffix-us0", us);

    if (ftemp >= 0)
    {
        us += sqrt(ftemp);
    }
    else
    {
        us += 0;
    }
    //v1ref + v0 + dv;
    //voltage compensation
    float qfilter = LP_Filter(q, 0.001, &qlold);
    dv = -PI(&pi_Phi, qfilter);
    // dv = MAX(dv, -9);

    float vref = us + dv;
    //wv *ACM.KE + 10 + dv;

    // CTRL.ual = vref * cos(ftheta);
    // CTRL.ube = vref * sin(ftheta);

    // dbglog("uffix-phi", phi);
    dbglog("uffix-p", p);
    dbglog("uffix-q", q);
    dbglog("uffix-qfilter", qfilter);
    dbglog("uffix-dv", dv);
    dbglog("uffix-hfp", hfp);
    dbglog("uffix-hfp0", hfp0);
    dbglog("uffix-dw", dw);
    dbglog("uffix-vout", vref);
    dbglog("uffix-wset", wv);
    dbglog("uffix-theta", (float)ftheta);
    dbglog("uffix-phi", phi);
    dbglog("uffix-us", vref);
    dbglog("uffix-ftemp", ftemp);

    return wv;
}

s32 ufcontrol0(double speed_cmd, double speed_cmd_dot)
{
    s32 K = 350;
    float fout = speed_cmd * MOTOR_POLES / 60;
    s16 ia = ACM.ial * fki;
    s16 ib = ACM.ibe * fki;

    s16 ua = CTRL.ual * fku;
    s16 ub = CTRL.ube * fku;

    s16 p = 3 / 2 * (ua * ia + ub * ib) >> 11;
    s16 hfp = p - s16LP_Filter(p, 3, &s16pOld);

    s16 wref = (u16)(2 * M_PI * fout * fkw); //speed_cmd;
    s16 dw = 0;

    if (wref != 0)
    {
        dw = (s16)((s64)K * hfp / wref);
    }

    dw = s16LP_Filter(dw, 328, &s32dwold);

    s16 q = 3 / 2 * (ub * ia - ua * ib) >> 11;
    s16 qfilter = s16LP_Filter(q, 33, &s32qlold);
    QRegulate.inputs.Ref = 0;
    QRegulate.inputs.Fdb = qfilter;
    PidReg_Calculate(&QRegulate);
    s16 dv = QRegulate.outputs.Out;

    s16 wv = wref - dw;
    s16 s16IsTotal = Math_Sqrt(ia * ia + ib * ib);
    u16 is_theta = iatan2(ib, ia);
    // u16 theta0 = (u16)(ftheta * DEGREE180 / M_PI);
    u16 thetaphi = theta - is_theta;

    s16 s16IsCosphi = (s16IsTotal * Math_Cos(thetaphi) >> 15);

    s16 s16IsFilted = s16LP_Filter(s16IsTotal, 328, &s16IsTotalOld);
    s16 s16IsCosFilted = s16LP_Filter(s16IsCosphi, 328, &s16IsCosphiOld);

    // wv = s16LP_Filter(wv, 328, &s32Wv);
    s32 s32Temp = ((s32)wv * u16Ke >> 10) * ((s32)wv * u16Ke >> 10);
    s32 s32Temp0 = ((s32)u16Rs * s16IsCosFilted >> 10) * ((s32)u16Rs * s16IsCosFilted >> 10);
    s32 s32Temp1 = ((s32)u16Rs * s16IsFilted >> 10) * ((s32)u16Rs * s16IsFilted >> 10);

    s32Temp = s32Temp + s32Temp0 - s32Temp1;

    s16 s16Us = (s16IsCosFilted * u16Rs >> 10);
    //   -dv;
    dbglog("uffix0-Uout0", s16Us / fku);
    if (s32Temp >= 0)
    {
        s16Us += Math_Sqrt(s32Temp);
    }

    theta += (s16)((s32)wv * 51 >> 11);
    /// 40; //40 = k_th/k_w/k_t

    if (s16Us < 100)
    {
        // s16Us = 100;
    }

    float ual = (float)(s16Us * Math_Cos(theta) >> 15) / fku;
    float ube = (float)(s16Us * Math_Sin(theta) >> 15) / fku;

    CTRL.ual = ual;
    CTRL.ube = ube;

    dbglog("uffix0-Uout", s16Us / fku);
    dbglog("uffix0-wv", wv / fkw);
    dbglog("uffix0-dw", dw / fkw);
    dbglog("uffix0-p", p / fkp);
    dbglog("uffix0-hfp", hfp / fkp);
    dbglog("uffix0-q", q / fkp);
    dbglog("uffix0-qfilter", qfilter / fkp);
    dbglog("uffix0-dv", dv / fku);
    dbglog("uffix0-theta", theta);
    dbglog("uffix0-IsTotal", s16IsTotal / fki);
    dbglog("uffix0-is_theta", theta_round((float)is_theta / DEGREE180 * M_PI));
    dbglog("uffix0-thetaphi", theta_round((float)thetaphi / DEGREE180 * M_PI));
    // dbglog("uffix0-is_theta", theta_round(is_theta));
    dbglog("uffix0-IsCosphi", s16IsCosphi / fki);
    dbglog("uffix0-IsFilted", s16IsFilted / fki);
    dbglog("uffix0-IsCosFilted", s16IsCosFilted / fki);
    dbglog("uffix0-s32Temp", s32Temp);
    dbglog("uffix0-s32Temp0", s32Temp0);
    dbglog("uffix0-s32Temp1", s32Temp1);
    dbglog("uffix0-ual", ual);
    dbglog("uffix0-ube", ube);

    return wv;
}

s32 ufcontrol(double speed_cmd, double speed_cmd_dot)
{
    float ua, ub, ia, ib;
    float K = 20;
    ua = CTRL.ual;
    ub = CTRL.ube;

    ia = ACM.ial;
    ib = ACM.ibe;

    float istotal = sqrt(ia * ia + ib * ib);
    float is_theta = theta_round(atan2(ib, ia));
    float thetaPhi = theta_round((float)theta / DEGREE180 * M_PI - is_theta);

    float iscosphi = istotal * cos(thetaPhi);

    dbglog("uffix-is", istotal);
    dbglog("uffix-iscosphi", iscosphi);
    dbglog("uffix-is_theta", is_theta);
    dbglog("uffix-thetaPhi", thetaPhi / M_PI * 180);

    float p = 3 / 2 * (ua * ia + ub * ib);
    float q = 3 / 2 * (ub * ia - ua * ib);
    float phi = atan2f(q, p);
    s16 dw = 0;

    if (phi > 0)
    {
        ia = 0;
    }
    float dphiv = -PI(&pi_Phi, phi);

    dbglog("uffix-phi", phi);
    dbglog("uffix-p", p);
    dbglog("uffix-q", q);
    dbglog("uffix-dv", dphiv);

    s32 wref0 = FP_SPEED2RAD(speed_cmd); //speed_cmd;
    s32 vout, vcomp, wset = 0;
    if (speed_cmd <= 0.1)
    {
        return 0;
    }

    u32DcRunTime++;
    if (u32DcRunTime < DC_BRAKE_TIME)
    {
        vout = 100;
        theta = 0;
    }
    else
    {
        if (wref < 5000)
        {
            u16RampStep = (RAMP_STEP >> 1);
        }
        else
        {
            u16RampStep = RAMP_STEP;
        }
        if (wsetold < ((s64)wref0 << 10))
        {
            wsetold += u16RampStep;
            wref = (wsetold >> 10);
        }
        else if (wsetold > ((s64)wref0 << 10))
        {
            wsetold -= u16RampStep;
            wref = (wsetold >> 10);
        }
        vcomp = (s16)((s64)wref * 1862 >> 16);
        /*     if (vcomp > FP_VOLTAGE(10.0))
    {
        vcomp = FP_VOLTAGE(10.0);
    } */

        if (vcomp > 78 * 2)
        {
            vcomp = 78 * 2;
        }
        else if (vcomp < 100)
        {
            vcomp = 100;
            //40 + 20 + 20 + 20;
            //100 / 2 / 2;
        }

        dbglog("uffix-vcomp", vcomp);

        float HFP = HighPassFilter_RC_1order(&p, &power_p, &dw_p, 16000);

        if (wref != 0)
        {
            dw = (s16)(-K / ((float)wref) * HFP);
        }
        dw = 0;
        dbglog("uffix-hfp", HFP);
        dbglog("uffix-dw", dw);

        wset = wref + (dw);

        if (wset < 1000)
        {
            // dphiv += 0.3 * p;
            vcomp = 100;
        }
        else
        {
            vcomp = 80;
        }
        // dphiv = 0;
        // dphiv = 0;
        // vcomp = (s32)(dphiv / 400 * 32768);
        // s32 vout = FP_VOLTAGE((float)wset * Fre_MAX * fke / 32768) + vcomp;
        if (dphiv < -vcomp)
        {
            dphiv = -vcomp;
        }
        dphiv = 0;

        vout = (s32)((s64)wset * 1257 >> 16) + dphiv + vcomp;

        // vout = (s32)(((s64)vout << 2) / 5);
        // theta += FP_THETA((float)wset * Fre_MAX / 32768 / PWM_FREQUENCY);
        // theta += (u16)((s32)wset * 261 >> 16);
    }

    float fwref = speed_cmd * ACM.npp / 60 * 2 * M_PI;
    theta += theta_round(fwref / 16000) / 2 / M_PI * 32768;
    //((float)wset / 163.689f);

    istotal = LIMIT(istotal, -3, 3);
    iscosphi = LIMIT(iscosphi, -3, 3);

    float isfilted = LP_Filter(istotal, 0.00001, &istotalold);
    float iscosfilted = LP_Filter(iscosphi, 0.00001, &iscosold);
    float ftemp = fwref * ACM.KE * fwref * ACM.KE;
    //    +ACM.R *iscosfilted *ACM.R *iscosfilted - ACM.R *isfilted *ACM.R *isfilted;

    float us = 0;
    //iscosphi *ACM.R;
    dbglog("uffix-us0", us);
    if (ftemp > 0)
    {
        us += sqrt(ftemp);
    }

    dbglog("uffix-us", us);
    vout = us / 400.0 / sqrt(3) * 32768;

    // theta = 65535;
    u8 u8TState = 0;

    // if (wref <= FP_SPEED2RAD(250))
    {
        CTRL.ual = MIN(FLOAT_V(vout * Math_Cos(theta) >> 15), 400);
        CTRL.ube = MIN(FLOAT_V(vout * Math_Sin(theta) >> 15), 400);
        bsmoInit = false;
    }
    /*     else
    {
        transfertime++;
        if (transfertime < 200)
        {
            CTRL.ual = FLOAT_V(vout * Math_Cos(theta) >> 15);
            CTRL.ube = FLOAT_V(vout * Math_Sin(theta) >> 15);
            bsmoInit = false;
        }
        else
        {
            transfertime = 1000;
            if (!bsmoInit)
            {
                ramp_set(250);
                bsmoInit = true;
                CTRL.pi_speed.i_state = 0;
                CTRL.pi_iMs.i_state = CTRL.iMs;
                CTRL.pi_iTs.i_state = CTRL.iTs;
                // CTRL.iTs_cmd = 0;
                fixsmo_transfer();
            }
        }
    } */

    sv1.Ualpha = CTRL.ual / 400.0 / sqrt(3) * 32768; //vout * Math_Cos(theta) >> 15;
    sv1.Ubeta = CTRL.ube / 400.0 / sqrt(3) * 32768;
    //vout *Math_Sin(theta) >> 15;
    Svpwm_Alpha_Belt_Calc(&sv1);
    Driver1.inputs.uwTa = sv1.Ta;
    Driver1.inputs.uwTb = sv1.Tb;
    Driver1.inputs.uwTc = sv1.Tc;
    PWM_Update(&Driver1);

    dbglog("uffix-vout", vout);

    dbglog("uffix-wset", wset);

    dbglog("uffix-theta", (float)theta / DEGREE180 * M_PI);
    // dbg_tst(25, wref);
    dbg_tst(15, theta);
    dbg_tst(14, FP_THETA(ACM.theta_d));
    dbg_tst(21, vcomp);

    return wset;
}