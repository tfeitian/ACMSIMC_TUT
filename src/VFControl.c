#include "controller.h"
#include "motor.h"
#include "tools.h"
#include "VFControl.h"
#include "math.h"

double wset, vout, theta = 0;
//- M_PI / 4;

float power_p, dw_p;

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

static int vc_count = 0;
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

    float power = 3 / 2 * (CTRL.ual * ia + CTRL.ube * ib);

    float HFP = HighPassFilter_RC_1order(&power, &power_p, &dw_p, 16000);
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
    vcomp = 15.0 / 200.0f * speed_cmd;
    if (vcomp > param[E_V_COMP])
    {
        vcomp = param[E_V_COMP];
    }
    dbg_tst(28, 0.3 * power);
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

    if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;
        CTRL.ual = vout * cos(theta);
        CTRL.ube = vout * sin(theta);
    }

    dbg_tst(23, dw);
    dbg_tst(24, wset);
    dbg_tst(25, wref);
    dbg_tst(15, theta);
}