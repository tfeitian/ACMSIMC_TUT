#include "userdefine.h"
#include "Fixpoint.h"
#include "controller.h"
#include "PI_Adjuster.h"
#include "motor.h"
#include "FixMath.h"
#include "tools.h"
#include "controller.h"

#define USE_HARNEFORS 1

PIDREG_OBJECT IdRegulate;
PIDREG_OBJECT IqRegulate;
PIDREG_OBJECT SpeedRegulate;

static u16 theta = 0;
static s16 wfb = 0, ia = 0, ib = 0, iq_cmd = 0;

static float theta_d_harnefors = 0;
static float w_harnefors = 0;

#define LAMBDA 2

static void fix_harnefors_svcm(float ud, float uq, float id, float iq);

void fix_vinit(void)
{
    PidReg_Intialize(&SpeedRegulate);
    /*Speed colsed loop  */
    SpeedRegulate.inputs.Ref = FP_SPEED2RAD(MAX_SPEED_RPM);
    SpeedRegulate.inputs.Kp = 8120;
    SpeedRegulate.inputs.Ki = 300;
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
    float fresult = finput * R_SHUNT * 32768; //ki_scale = ADC_SAMPLE_VALUE * 8
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
    wfb = FP_RAD(ACM.omg);
#if USE_HARNEFORS == 1
    theta = FP_THETA(theta_d_harnefors);
    wfb = FP_RAD(w_harnefors);
#endif
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

        wref = FP_SPEED2RAD(speed_cmd);
        SpeedRegulate.inputs.Fdb = wfb;
        SpeedRegulate.inputs.Ref = wref;
        dbg_tst(17, wref - wfb);
        PidReg_Calculate(&SpeedRegulate);
        if (speed_cmd > 0)
        {
            iq_cmd = 1000;
        }
        if (speed_cmd >= 200)
        {
            iq_cmd = 0;
        }
        iq_cmd = SpeedRegulate.outputs.Out;

        iq_cmd = FP_CURRENT(-PI(&sPi_Speed, ACM.omg - speed_cmd * RPM_2_RAD_PER_SEC(ACM.npp)));
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

#if FIX_IQ_PID == 1
    IdRegulate.inputs.Fdb = id;
    IdRegulate.inputs.Ref = id_cmd;
    IqRegulate.inputs.Fdb = iq;
    IqRegulate.inputs.Ref = iq_cmd;

    PidReg_Calculate(&IdRegulate);
    PidReg_Calculate(&IqRegulate);
    ud = IdRegulate.outputs.Out;
    uq = IqRegulate.outputs.Out;
#else
    double fud, fuq, fid, fiq;

    double a = (double)FLOAT_I(iq - iq_cmd);

    fud = -PI(&sPi_Id, (double)FLOAT_I(id - id_cmd));
    fuq = -PI(&sPi_Iq, a);
    ud = FP_VOLTAGE(fud);
    uq = FP_VOLTAGE(fuq);
#endif
    dbg_tst(11, ud);
    dbg_tst(12, uq);

    fp_dqtoab(ud, uq, theta, &ua, &ub);
    CTRL.ual = VOLTAGE_U16_TO_FLOAT(ua);
    CTRL.ube = VOLTAGE_U16_TO_FLOAT(ub);

    fix_harnefors_svcm(FLOAT_V(ud), FLOAT_V(uq), FLOAT_I(id), FLOAT_I(iq));
    // fix_harnefors_svcm(-7.089844, 67.031250, -0.936890, -0.003052);
    dbg_tst(28, FP_THETA(theta_d_harnefors));
    dbg_tst(29, FP_RAD(w_harnefors));
}

#define LAMBDA 2        // 2
#define CJH_TUNING_A 1  // 1
#define CJH_TUNING_B 1  // 1
#define KE_MISMATCH 1.0 // 0.7
#define L_COEFF 1       //0.8
#define R_COEFF 1       //1.03

void fix_harnefors_svcm(float ud, float uq, float id, float iq)
{
    // printf("%f  %f  %f  %f\n", ud, uq, id, iq);
    s16 lambda_s = LAMBDA * fix_sign(theta_d_harnefors);
    // s16 alpha_bw_lpf = CJH_TUNING_A *
    double d_axis_emf;
    double q_axis_emf;

    double alpha_bw_lpf = CJH_TUNING_A * 0.1 * (1000 * RPM_2_RAD_PER_SEC(ACM.npp)) + CJH_TUNING_B * 2 * LAMBDA * fabs(w_harnefors);
    d_axis_emf = ud - 1 * CTRL.R * R_COEFF * id + w_harnefors * 1.0 * L_COEFF * CTRL.Lq * iq; // eemf
    q_axis_emf = uq - 1 * CTRL.R * R_COEFF * iq - w_harnefors * 1.0 * CTRL.Lq * id;           // eemf
    w_harnefors += TS * alpha_bw_lpf * ((q_axis_emf - lambda_s * d_axis_emf) / (CTRL.KE * KE_MISMATCH) - w_harnefors);
    theta_d_harnefors += TS * w_harnefors;

    if (theta_d_harnefors > M_PI)
    {
        theta_d_harnefors -= 2 * M_PI;
    }
    if (theta_d_harnefors < -M_PI)
    {
        theta_d_harnefors += 2 * M_PI;
    }
}