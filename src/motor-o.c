#include "ACMSim.h"
#include "motor.h"
#include "tools.h"

#if MACHINE_TYPE == INDUCTION_MACHINE
struct InductionMachineSimulated ACM;
#define NUMBER_OF_STATES 6
#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
struct SynchronousMachineSimulated ACM;
#define NUMBER_OF_STATES 4
#endif
#define NS NUMBER_OF_STATES

static double xx[5];

static int isNumber(double x)
{
    // This looks like it should always be true,
    // but it's false if x is a NaN (1.#QNAN0).
    return (x == x);
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}

static double ld_matching(double isd)
{
    double coef[4] = {0.1506734, -0.40800866, -6.12794613, 40.83116883};
    //a *x * * 2 + b *x + c
    // isd = 0;
    // return ACM.Lq * 1.2;

    return (isd * isd * isd * coef[0] + isd * isd * coef[1] + isd * coef[2] + coef[3]) * (ACM.Lq / 40.83116883) * 1.05;
}

static void rK5_dynamics(double t, double *x, double *fx, double ld)
{
#if MACHINE_TYPE == INDUCTION_MACHINE
    // electromagnetic model
    fx[2] = ACM.rreq * x[0] - ACM.alpha * x[2] - x[4] * x[3]; // flux-alpha
    fx[3] = ACM.rreq * x[1] - ACM.alpha * x[3] + x[4] * x[2]; // flux-beta
    fx[0] = (ACM.ual - ACM.rs * x[0] - fx[2]) / ACM.Lsigma;   // current-alpha
    fx[1] = (ACM.ube - ACM.rs * x[1] - fx[3]) / ACM.Lsigma;   // current-beta

    // mechanical model
    ACM.Tem = ACM.npp * (x[1] * x[2] - x[0] * x[3]);
    fx[4] = (ACM.Tem - ACM.Tload) * ACM.mu_m; // elec. angular rotor speed
    fx[5] = x[4];                             // elec. angular rotor position

#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
    // electromagnetic model
    fx[0] = (ACM.ud - ACM.R * x[0] + x[2] * ACM.Lq * x[1]) / ld;

    fx[1] = (ACM.uq - ACM.R * x[1] - x[2] * ld * x[0] - x[2] * ACM.KE) / ACM.Lq;

    // mechanical model
    ACM.Tem = ACM.npp * (x[1] * ACM.KE + (ld - ACM.Lq) * x[0] * x[1]);
    fx[2] = (ACM.Tem - ACM.Tload) * ACM.mu_m; // elec. angular rotor speed
    fx[3] = x[2];                             // elec. angular rotor position
#endif
}

static void rK555_Lin(double t, double *x, double hs, double ld)
{
    double k1[NS], k2[NS], k3[NS], k4[NS], xk[NS];
    double fx[NS];
    int i;

    rK5_dynamics(t, x, fx, ld); // timer.t,
    for (i = 0; i < NS; ++i)
    {
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i] * 0.5;
    }

    rK5_dynamics(t, xk, fx, ld); // timer.t+hs/2.,
    for (i = 0; i < NS; ++i)
    {
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i] * 0.5;
    }

    rK5_dynamics(t, xk, fx, ld); // timer.t+hs/2.,
    for (i = 0; i < NS; ++i)
    {
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }

    rK5_dynamics(t, xk, fx, ld); // timer.t+hs,
    for (i = 0; i < NS; ++i)
    {
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2 * (k2[i] + k3[i]) + k4[i]) / 6.0;
    }
}
////////////////////////////////////////////////////////////////////////////////////////
void Machine_init()
{
#if MACHINE_TYPE == INDUCTION_MACHINE
    int i;
    for (i = 0; i < 5; ++i)
    {
        ACM.x[i] = 0.0;
    }
    ACM.rpm = 0.0;
    ACM.rpm_cmd = 0.0;
    ACM.rpm_deriv_cmd = 0.0;
    ACM.Tload = 0.0;
    ACM.Tem = 0.0;

    ACM.Lmu = 0.4482;
    ACM.Lsigma = 0.0126;

    ACM.rreq = 1.69;
    ACM.rs = 3.04;

    ACM.alpha = ACM.rreq / (ACM.Lmu);
    ACM.Lmu_inv = 1.0 / ACM.Lmu;

    ACM.Js = 0.0636; // Awaya92 using im.omg
    ACM.npp = 2;
    ACM.mu_m = ACM.npp / ACM.Js;

    ACM.Ts = MACHINE_TS;

    ACM.ial = 0.0;
    ACM.ibe = 0.0;

    ACM.ual = 0.0;
    ACM.ube = 0.0;
#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
    int i;
    for (i = 0; i < 5; ++i)
    {
        xx[i] = 0.0;
    }
    ACM.rpm = 0.0;
    ACM.rpm_deriv_cmd = 0.0;
    ACM.Tload = 0.0;
    ACM.Tem = 0.0;

    // double motorData[4] = {10.3, 87.5, 87.5, 125}; //efan
    // double motorData[5] = {0.45, 4.15, 16.74, 0.504 / 13.5 * 2 * 1000, 2}; //Origin motor data
    double motorData[5] = {7.6, 75, 75, 150, 5}; //138mm
    ACM.R = motorData[0];
    ACM.Ld = motorData[1] * 1e-3 * 0.99;
    ACM.Lq = motorData[2] * 1e-3;
    ACM.Bemf = motorData[3];
    //0.3375; // = backemf_rms@1000rpm/1000/pole pairs*13.5
    //0.504; // Vs/rad
    ACM.L0 = 0.5 * (ACM.Ld + ACM.Lq);
    ACM.L1 = 0.5 * (ACM.Ld - ACM.Lq);

    ACM.Js = 0.06; // Awaya92 using ACM.omg
    ACM.npp = motorData[4];
    ACM.mu_m = ACM.npp / ACM.Js;
    ACM.KE = 13.5 * ACM.Bemf / 1000.0 / ACM.npp; //phif

    ACM.Ts = MACHINE_TS;

    ACM.id = 0.0;
    ACM.iq = 0.0;

    ACM.ial = 0.0;
    ACM.ibe = 0.0;

    ACM.ud = 0.0;
    ACM.uq = 0.0;

    ACM.ual = 0.0;
    ACM.ube = 0.0;

    ACM.theta_d = param[E_MOTOR_ANGLE] * M_PI / 180.0f;
    xx[3] = ACM.theta_d;
#endif
}

int machine_simulation(double ud, double uq)
{
    ACM.ud = ud;
    ACM.uq = uq;
    rK555_Lin(0, xx, ACM.Ts, ACM.Ld);

// API for explicit access
#if MACHINE_TYPE == INDUCTION_MACHINE
    ACM.ial = ACM.x[0];
    ACM.ibe = ACM.x[1];
    ACM.rpm = ACM.x[4] * 60 / (2 * M_PI * ACM.npp);

#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
    if (xx[3] > M_PI)
    {
        xx[3] -= 2 * M_PI;
    }
    else if (xx[3] < -M_PI)
    {
        xx[3] += 2 * M_PI; // 坝转
    }
    ACM.theta_d = xx[3];

    ACM.id = xx[0];
    ACM.iq = xx[1];
    ACM.phid = ACM.id * ACM.Ld + ACM.KE;
    ACM.phiq = ACM.iq * ACM.Lq;

    ACM.ial = MT2A(ACM.id, ACM.iq, cos(ACM.theta_d), sin(ACM.theta_d));
    ACM.ibe = MT2B(ACM.id, ACM.iq, cos(ACM.theta_d), sin(ACM.theta_d));
    ACM.rpm = xx[2] * 60 / (2 * M_PI * ACM.npp);
    ACM.omg = xx[2];
    ACM.Ea = MT2A(ACM.ud, ACM.uq, cos(ACM.theta_d), sin(ACM.theta_d)) - ACM.R * ACM.ial;
    ACM.Eb = MT2B(ACM.ud, ACM.uq, cos(ACM.theta_d), sin(ACM.theta_d)) - ACM.R * ACM.ibe;
    //  -ACM.Ld *xx;
    dbg_tst(19, ACM.phid - ACM.KE);
    dbg_tst(20, ACM.phiq);
    dbg_tst(21, atan2f(-ACM.Ea, ACM.Eb));
    dbg_tst(22, ACM.id);
    dbg_tst(23, ACM.iq);

#endif

    if (isNumber(ACM.rpm))
    {
        return false;
    }
    else
    {
        printf("ACM.rpm is %g\n", ACM.rpm);
        return true;
    }
}