#include "userdefine.h"
#include "Fixpoint.h"
#include "controller.h"
#include "PI_Adjuster.h"
#include "motor.h"
#include "MatrixConvert.h"
#include "tools.h"

PIDREG_OBJECT IdRegulate;
PIDREG_OBJECT IqRegulate;
PIDREG_OBJECT SpeedRegulate;

static u16 theta = 0;
static s16 wfb = 0, ia = 0, ib = 0, iq_cmd = 0;

#define LAMBDA 2

void fix_vinit(void)
{
    PidReg_Intialize(&SpeedRegulate);
    /*Speed colsed loop  */
    SpeedRegulate.inputs.Ref = FP_SPEED(MAX_SPEED_RPM);
    SpeedRegulate.inputs.Kp = 5120;
    SpeedRegulate.inputs.Ki = 30;
    //(SpeedRegulate.inputs.Kp * 4.77) / 5 *
    (TS * VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE);
    SpeedRegulate.inputs.Kc = 32768 / 4;
    SpeedRegulate.OutMax = FP_CURRENT(MAX_CURRENT_PEAK);
    SpeedRegulate.OutMin = 0;
    PidReg_Intialize(&IdRegulate);
    PidReg_Intialize(&IqRegulate);

    /*D axis current */
    IdRegulate.inputs.Kp = 10.0;
    IdRegulate.inputs.Ki = 10;
    IdRegulate.inputs.Ref = 0;
    IdRegulate.Ui = 0;
    IdRegulate.SatErr = 0;
    IdRegulate.OutMax = 20000;
    IdRegulate.OutMin = 0 - IdRegulate.OutMax;
    /*Q axis current */
    IqRegulate.inputs.Kp = IdRegulate.inputs.Kp;
    IqRegulate.inputs.Ki = 20;
    IqRegulate.Ui = 0;
    IqRegulate.SatErr = 0;
    IqRegulate.OutMax = 27000;
    IqRegulate.OutMin = 0 - IqRegulate.OutMax;
}

s16 FP_CURRENT(float finput)
{
    float fresult = finput * R_SHUNT * 32768;
    if (fresult > 32767)
    {
        fresult = 32767;
    }
    else if (fresult < -32768)
    {
        fresult = -32768;
    }
    return (s16)fresult;
}
void fix_measure()
{
    theta = FP_THETA(ACM.theta_d);
    wfb = FP_SPEED(ACM.omg);
    ia = FP_CURRENT(ACM.ial);
    ib = FP_CURRENT(ACM.ibe);
}

s16 fix_sign(s16 s16w)
{
    if (s16w >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}

#define VOLTAGE_U16_TO_FLOAT(x) (x * VDC_DIV_COFF * 3.3 / 4096)

void fix_vControl(double speed_cmd, double speed_cmd_dot)
{
    static int vc_count = 0;
    s16 id = 0, iq = 0, id_cmd = 0;
    s32 ud = 0, uq = 0, ua = 0, ub = 0;
    s64 wref = 0;

    fix_measure();
    //speed loop
    if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;

        wref = FP_SPEED(speed_cmd);
        SpeedRegulate.inputs.Fdb = wfb;
        SpeedRegulate.inputs.Ref = wref;
        dbg_tst(17, wref - wfb);
        PidReg_Calculate(&SpeedRegulate);
        if (speed_cmd > 0)
        {
            // iq_cmd = 1000;
        }
        iq_cmd = SpeedRegulate.outputs.Out;
        dbg_tst(24, wref);
        dbg_tst(25, wfb);
    }
    id_cmd = iq_cmd / LAMBDA * fix_sign(wfb);

    fp_abtodq(ia, ib, theta, &id, &iq);
    dbg_tst(22, id_cmd);
    dbg_tst(23, iq_cmd);

    dbg_tst(15, theta);

    dbg_tst(13, id);
    dbg_tst(14, iq);

    IdRegulate.inputs.Fdb = id;
    IdRegulate.inputs.Ref = id_cmd;
    IqRegulate.inputs.Fdb = iq;
    IqRegulate.inputs.Ref = iq_cmd;

    PidReg_Calculate(&IdRegulate);
    PidReg_Calculate(&IqRegulate);

    ud = IdRegulate.outputs.Out;
    uq = IqRegulate.outputs.Out;
    dbg_tst(11, ud);
    dbg_tst(12, uq);

    fp_dqtoab(ud, uq, theta, &ua, &ub);
    CTRL.ual = VOLTAGE_U16_TO_FLOAT(ua);
    CTRL.ube = VOLTAGE_U16_TO_FLOAT(ub);
}
