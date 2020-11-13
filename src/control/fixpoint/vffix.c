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

static float power_p, dw_p;
u16 theta;
static int vc_count = 0;
s16 wset = 0;

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

#define RAMP_STEP 1024 / 5

u16 u16RampStep = 0;

s16 wref = 0;

s32 wsetold = 0;

#define DC_BRAKE_TIME 0.5 * 16000

u32 u32DcRunTime = 0;

extern SVGENAB sv1;

bool bsmoInit = false;

static u16 transfertime = 0;

s32 ufcontrol(double speed_cmd, double speed_cmd_dot)
{
    s16 dw = 0;
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
            u16RampStep = (RAMP_STEP >>1);
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
        }

        wset = wref + FP_RAD(dw);
        // s32 vout = FP_VOLTAGE((float)wset * Fre_MAX * fke / 32768) + vcomp;
        vout = (s32)((s64)wset * 1257 >> 16) + vcomp;

        // vout = (s32)(((s64)vout << 2) / 5);
        // theta += FP_THETA((float)wset * Fre_MAX / 32768 / PWM_FREQUENCY);
        theta += (u16)((s32)wset * 261 >> 16);
    }

    // theta = 65535;
    u8 u8TState = 0;

    // if (wref <= FP_SPEED2RAD(250))
    {
        CTRL.ual = FLOAT_V(vout * Math_Cos(theta) >> 15);
        CTRL.ube = FLOAT_V(vout * Math_Sin(theta) >> 15);
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

    dbglog("uffix-wset", FLOAT_RAD(wset));
    // dbg_tst(25, wref);
    dbg_tst(15, theta);
    dbg_tst(14, FP_THETA(ACM.theta_d));
    dbg_tst(21, vcomp);

    return bsmoInit;
}