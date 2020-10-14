#include "controller.h"
#include "motor.h"
#include "tools.h"
#include "VFControl.h"
#include "math.h"

double wset, vout, theta = 0;
//- M_PI / 4;

double HighPassFilter_RC_1order(double *Vi, double *Vi_p, double *Vo_p, double sampleFrq)
{
    double Vo;
    double CutFrq, RC, Coff;

    //high pass filter @cutoff frequency = 0.5 Hz
    CutFrq = 50;
    RC = (double)1.0 / 2.0 / M_PI / CutFrq;
    Coff = RC / (RC + 1 / sampleFrq);
    Vo = ((*Vi) - (*Vi_p) + (*Vo_p)) * Coff;

    //update
    *Vo_p = Vo;
    *Vi_p = *Vi;

    return Vo;
}

double power_p, dw_p;

static int pretime = 1;
static double precnt = 0;

bool pre_run(double speed_cmd, double speed_cmd_dot)
{
    if (speed_cmd > 0 && precnt < pretime)
    {
        precnt += TS;
        vout = 0.003;
        theta = 0;

        CTRL.ual = vout * cos(theta);
        CTRL.ube = vout * sin(theta);
        return false;
    }
    return true;
}

void vf_control(double speed_cmd, double speed_cmd_dot)
{
    double K = 20;
    double wref = RPM_2_RAD_PER_SEC(speed_cmd);

    double ia = ACM.ial;
    double ib = ACM.ibe;

    double is = sqrt(ia * ia + ib * ib);

    double isOld;
    double is_filt = LP_Filter(is, 0.0000001, &isOld);
    dbg_tst(16, is);
    dbg_tst(18, is_filt);

    double is_theta = rounddegree(atan2(ib, ia));
    double p_angle = rounddegree(theta - is_theta);

    dbg_tst(14, is_theta);
    dbg_tst(13, p_angle);

    double power = 3 / 2 * (CTRL.ual * ia + CTRL.ube * ib);

    double HFP = HighPassFilter_RC_1order(&power, &power_p, &dw_p, 16000);
    dbg_tst(11, power);
    dbg_tst(12, HFP);

    double dw = 0;
    if (wref != 0)
    {
        dw = -K / wref * HFP;
    }

    wset = wref + dw;
    // double wr = wset
    double vcomp = 0;
    if (wset < 4)
    {
        // vcomp = 1;
    }

    double a, b, c, d;

    a = wset * ACM.KE;
    b = ACM.R * is_filt * cos(p_angle);
    c = ACM.R * is_filt;

    d = MAX(0, a * a + b * b - c * c);

    dbg_tst(20, c);
    vcomp = 10.0 / 200.0f * speed_cmd;
    vout = a + vcomp;

    // vout = b + a;
    dbg_tst(21, vcomp);

    /*     if (d > 0)
    {

        vout = b + sqrt(d);
    }
    else
    {
        vout = b;
    } */

    if (!isNumber(vout))
    {
        vout = 0;
    }
    // vout = b + sqrt(d);
    dbg_tst(17, vout);

    theta += wset * TS;

    theta = rounddegree(theta);

    CTRL.ual = vout * cos(theta);
    CTRL.ube = vout * sin(theta);
    if (!isNumber(CTRL.ual))
    {
        vout = 0;
    }
    if (!isNumber(CTRL.ube))
    {
        vout = 0;
    }
    dbg_tst(23, dw);
    dbg_tst(24, wset);
    dbg_tst(25, wref);
    dbg_tst(15, theta);
}