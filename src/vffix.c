#include "controller.h"
#include "motor.h"
#include "tools.h"
#include "VFControl.h"
#include "math.h"
#include "Fixpoint.h"
#include "Fixmath.h"

static float power_p, dw_p;
u16 theta;
static int vc_count = 0;

void vffix_control(double speed_cmd, double speed_cmd_dot)
{
    s16 dw = 0;
    s16 wref = FP_SPEED(speed_cmd);

    if (speed_cmd >= 200)
    {
        dw = 0;
    }

    float ia = ACM.ial;
    float ib = ACM.ibe;
    float K = 20;

    float power = 3 / 2 * (CTRL.ual * ia + CTRL.ube * ib);
    float HFP = HighPassFilter_RC_1order(&power, &power_p, &dw_p, 16000);

    s16 wset = wref + FP_SPEED(dw);
    dw = -K / ((float)wset * Freq_MAX / 32768) * HFP;

    s16 vcomp = FP_VOLTAGE(15.0 / 200.0 * speed_cmd);
    s32 vout = FP_VOLTAGE((float)wset * Freq_MAX * ACM.KE / 32768) + vcomp;

    theta += FP_THETA((float)wset * Freq_MAX / 32768 * TS);

    if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;
        CTRL.ual = FLOAT_V(vout * Math_Cos(theta) >> 15);
        CTRL.ube = FLOAT_V(vout * Math_Sin(theta) >> 15);
    }

    dbg_tst(17, vout);
    dbg_tst(23, dw);
    dbg_tst(24, wset);
    dbg_tst(25, wref);
    dbg_tst(15, theta);
    dbg_tst(14, FP_THETA(ACM.theta_d));
    dbg_tst(21, vcomp);
    dbg_tst(11, power);
    dbg_tst(12, HFP);
}