#include "controller.h"
#include "ACMSim.h"
#include "math.h"
#include "motor.h"
#include "observer.h"
#include "tools.h"
#include "string.h"
#include "userdefine.h"

#define USING_HARNEFORS 0
/* PI Control
 * */
double PI(struct PI_Reg *r, double err)
{
    double output;
    r->i_state += err * r->Ki;   // 积分
    if (r->i_state > r->i_limit) // 添加积分饱和特性
        r->i_state = r->i_limit;
    else if (r->i_state < -r->i_limit)
        r->i_state = -r->i_limit;

    output = r->i_state + err * r->Kp;

    if (output > r->i_limit)
        output = r->i_limit;
    else if (output < -r->i_limit)
        output = -r->i_limit;
    return output;
}
float rounddegree(float xx)
{
    if (xx > M_PI)
    {
        xx -= 2 * M_PI;
    }
    else if (xx < -M_PI)
    {
        xx += 2 * M_PI; // 反转！
    }
    return xx;
}
static double PI_Degree(struct PI_Reg *r, double err)
{
    double output;
    r->i_state += err * r->Ki;   // 积分
    if (r->i_state > r->i_limit) // 添加积分饱和特性
        r->i_state = r->i_limit;
    else if (r->i_state < -r->i_limit)
        r->i_state = -r->i_limit;

    output = r->i_state + err * r->Kp;

    /*     if (output > r->i_limit)
        output = r->i_limit;
    else if (output < -r->i_limit)
        output = -r->i_limit; */
    output = rounddegree(output);
    r->i_state = output - err * r->Kp;
    return output;
}
#if MACHINE_TYPE == INDUCTION_MACHINE
/* Initialization */
struct ControllerForExperiment CTRL;
void CTRL_init()
{
    int i = 0, j = 0;

    CTRL.timebase = 0.0;

    /* Parameter (including speed) Adaptation */
    CTRL.rs = ACM.rs;
    CTRL.rreq = ACM.rreq;
    CTRL.Lsigma = ACM.Lsigma;
    CTRL.alpha = ACM.alpha;
    CTRL.Lmu = ACM.Lmu;
    CTRL.Lmu_inv = 1.0 / ACM.Lmu;
    CTRL.Js = ACM.Js;
    CTRL.Js_inv = 1.0 / ACM.Js;

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    CTRL.rpm_cmd = 0.0;
    CTRL.rotor_flux_cmd = 0.5;

    CTRL.omg_ctrl_err = 0.0;
    CTRL.speed_ctrl_err = 0.0;

    CTRL.omg_fb = 0.0;
    CTRL.ial_fb = 0.0;
    CTRL.ibe_fb = 0.0;
    CTRL.psi_mu_al_fb = 0.0;
    CTRL.psi_mu_be_fb = 0.0;

    CTRL.theta_M = 0.0;
    CTRL.cosT = 0.0;
    CTRL.sinT = 1;

    CTRL.uMs_cmd = 0.0;
    CTRL.uTs_cmd = 0.0;
    CTRL.iMs_cmd = 0.0;
    CTRL.iTs_cmd = 0.0;

    CTRL.omega_syn = 0.0;
    CTRL.omega_sl = 0.0;

    // ver. IEMDC
    CTRL.pi_speed.Kp = 0.5;
    CTRL.pi_speed.Ti = 5;
    CTRL.pi_speed.Ki = (CTRL.pi_speed.Kp * 4.77) / CTRL.pi_speed.Ti *
                       (TS * VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE);
    CTRL.pi_speed.i_state = 0.0;
    CTRL.pi_speed.i_limit = 8;

    printf("Kp_omg=%g, Ki_omg=%g\n", CTRL.pi_speed.Kp, CTRL.pi_speed.Ki);

    CTRL.pi_iMs.Kp = 15; // cutoff frequency of 1530 rad/s
    CTRL.pi_iMs.Ti = 0.08;
    CTRL.pi_iMs.Ki = CTRL.pi_iMs.Kp / CTRL.pi_iMs.Ti * TS; // =0.025
    CTRL.pi_iMs.i_state = 0.0;
    CTRL.pi_iMs.i_limit = 350; // 350.0; // unit: Volt

    CTRL.pi_iTs.Kp = 15;
    CTRL.pi_iTs.Ti = 0.08;
    CTRL.pi_iTs.Ki = CTRL.pi_iTs.Kp / CTRL.pi_iTs.Ti * TS;
    CTRL.pi_iTs.i_state = 0.0;
    CTRL.pi_iTs.i_limit = 650; // unit: Volt, 350V->max 1300rpm

    printf("Kp_cur=%g, Ki_cur=%g\n", CTRL.pi_iMs.Kp, CTRL.pi_iMs.Ki);
}
void control(double speed_cmd, double speed_cmd_dot)
{
// OPEN LOOP CONTROL
#if CONTROL_STRATEGY == VVVF_CONTROL
#define VF_RATIO 18  // 18.0 // 8 ~ 18 shows saturated phenomenon
    double freq = 2; // 0.15 ~ 0.5 ~ 2 （0.1时电压李萨茹就变成一个圆了）
    double volt = VF_RATIO * freq;
    CTRL.ual = volt * cos(2 * M_PI * freq * CTRL.timebase);
    CTRL.ube = volt * sin(2 * M_PI * freq * CTRL.timebase);
    return
#endif

// Input 1 is feedback: estimated speed or measured speed
#if SENSORLESS_CONTROL
        CTRL.omg_fb = ob.tajima.omg;
    CTRL.omega_syn = ob.tajima.omega_syn;
    CTRL.omega_sl = ob.tajima.omega_sl;
#else
    CTRL.omg_fb = im.omg;
#endif
    // Input 2 is feedback: measured current
    CTRL.ial_fb = IS_C(0);
    CTRL.ibe_fb = IS_C(1);
// Input 3 differs for DFOC and IFOC
#if CONTROL_STRATEGY == DFOC
    // DFOC: estimated flux components in alpha-beta frame
    CTRL.psi_mu_al_fb = ob.psi_mu_al;
    CTRL.psi_mu_be_fb = ob.psi_mu_be;
#elif CONTROL_STRATEGY == IFOC
    // IFOC: estimated rotor resistance
    CTRL.rreq = ob.rreq;
#else
#endif

    // Flux (linkage) command
    CTRL.rotor_flux_cmd = 0.5; // f(speed, dc bus voltage, last torque current command)
                               // 1. speed is compared with the base speed to decide flux weakening
                               // or not
                               // 2. dc bus voltage is required for certain application
                               // 3. last torque current command is required for loss minimization

    // M-axis current command
    CTRL.iMs_cmd = CTRL.rotor_flux_cmd * CTRL.Lmu_inv +
                   M1 * OMG1 * cos(OMG1 * CTRL.timebase) / CTRL.rreq;
    // printf("%g, %g, %g\n", CTRL.Lmu_inv, CTRL.iMs_cmd, CTRL.iTs_cmd);

    // T-axis current command
    static int vc_count = 0;
    if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;
        CTRL.omg_ctrl_err = CTRL.omg_fb - speed_cmd * RPM_2_RAD_PER_SEC;
        CTRL.iTs_cmd = -PI(&CTRL.pi_speed, CTRL.omg_ctrl_err);

        CTRL.speed_ctrl_err = CTRL.omg_ctrl_err * RAD_PER_SEC_2_RPM(ACM.npp);
    }

#if CONTROL_STRATEGY == DFOC
    // feedback field orientation
    double modulus = sqrt(CTRL.psi_mu_al_fb * CTRL.psi_mu_al_fb +
                          CTRL.psi_mu_be_fb * CTRL.psi_mu_be_fb);
    if (modulus < 1e-3)
    {
        CTRL.cosT = 0;
        CTRL.sinT = 1;
    }
    else
    {
        CTRL.cosT = CTRL.psi_mu_al / modulus;
        CTRL.sinT = CTRL.psi_mu_be / modulus;
    }
#elif CONTROL_STRATEGY == IFOC
    // Feed-forward field orientation
    CTRL.theta_M += TS * CTRL.omega_syn;

    if (CTRL.theta_M > M_PI)
    {
        CTRL.theta_M -= 2 * M_PI;
    }
    else if (CTRL.theta_M < -M_PI)
    {
        CTRL.theta_M += 2 * M_PI; // 反转！
    }

    CTRL.omega_sl = CTRL.rreq * CTRL.iTs_cmd / CTRL.rotor_flux_cmd;
    CTRL.omega_syn = CTRL.omg_fb + CTRL.omega_sl;

    CTRL.cosT = cos(CTRL.theta_M);
    CTRL.sinT = sin(CTRL.theta_M);
#endif

    // Measured current in M-T frame
    CTRL.iMs = AB2M(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);
    CTRL.iTs = AB2T(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);

    // Voltage command in M-T frame
    double vM, vT;
    vM = -PI(&CTRL.pi_iMs, CTRL.iMs - CTRL.iMs_cmd);
    vT = -PI(&CTRL.pi_iTs, CTRL.iTs - CTRL.iTs_cmd);

    // Current loop decoupling (skipped, see Chen.Huang-Stable)
    CTRL.uMs_cmd = vM;
    CTRL.uTs_cmd = vT;

    // Voltage command in alpha-beta frame
    CTRL.ual = MT2A(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
}

#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
static double Vinj = 0;
static double whfi = 400 * 2 * M_PI;
static double theta_hfi = 0;

struct PI_Reg sPi_Speed;
struct PI_Reg sPi_Id;
struct PI_Reg sPi_Iq;

static double HFI_Voltage(float dtime)
{
    theta_hfi += whfi * dtime;
    theta_hfi = rounddegree(theta_hfi);
    return Vinj * sin(theta_hfi);
}
/* Initialization */
struct ControllerForExperiment CTRL;
void CTRL_init()
{
    int i = 0, j = 0;

    CTRL.timebase = 0.0;

    CTRL.ual = 0.0;
    CTRL.ube = 0.0;

    CTRL.R = ACM.R;
    CTRL.KE = ACM.KE;
    CTRL.Ld = ACM.Ld;
    CTRL.Lq = ACM.Lq;

    CTRL.Tload = 0.0;
    CTRL.rpm_cmd = 0.0;

    CTRL.Js = ACM.Js;
    CTRL.Js_inv = 1.0 / CTRL.Js;

    CTRL.omg_fb = 0.0;
    CTRL.ial_fb = 0.0;
    CTRL.ibe_fb = 0.0;
    CTRL.psi_mu_al_fb = 0.0;
    CTRL.psi_mu_be_fb = 0.0;

    CTRL.rotor_flux_cmd = 0.0; // id=0 control

    CTRL.omg_ctrl_err = 0.0;
    CTRL.speed_ctrl_err = 0.0;

    CTRL.iMs = 0.0;
    CTRL.iTs = 0.0;

    CTRL.theta_M = 0.0;
    CTRL.cosT = 1.0;
    CTRL.sinT = 0.0;

    CTRL.omega_syn = 0.0;

    CTRL.uMs_cmd = 0.0;
    CTRL.uTs_cmd = 0.0;
    CTRL.iMs_cmd = 0.0;
    CTRL.iTs_cmd = 0.0;

    // ver. IEMDC
    CTRL.pi_speed.Kp = 0.5;
    CTRL.pi_speed.Ti = 5;
    CTRL.pi_speed.Ki = (CTRL.pi_speed.Kp * 4.77) / CTRL.pi_speed.Ti *
                       (TS * VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE);
    CTRL.pi_speed.i_state = 0.0;
    CTRL.pi_speed.i_limit = 8;
    memcpy(&sPi_Speed, &CTRL.pi_speed, sizeof(sPi_Speed));

    printf("Kp_omg=%g, Ki_omg=%g\n", CTRL.pi_speed.Kp, CTRL.pi_speed.Ki);

    CTRL.pi_iMs.Kp = 1300; // cutoff frequency of 1530 rad/s
    CTRL.pi_iMs.Ti = ACM.L0 / ACM.R;
    CTRL.pi_iMs.Ki = CTRL.pi_iMs.Kp / CTRL.pi_iMs.Ti * TS; // =0.025
    CTRL.pi_iMs.i_state = 0.0;
    CTRL.pi_iMs.i_limit = 350; // 350.0; // unit: Volt

    memcpy(&sPi_Id, &CTRL.pi_iMs, sizeof(sPi_Id));

    CTRL.pi_iTs.Kp = 1300;
    CTRL.pi_iTs.Ti = ACM.L0 / ACM.R;
    CTRL.pi_iTs.Ki = CTRL.pi_iTs.Kp / CTRL.pi_iTs.Ti * TS;
    CTRL.pi_iTs.i_state = 0.0;
    CTRL.pi_iTs.i_limit = 350; // unit: Volt, 350V->max 1300rpm
    memcpy(&sPi_Iq, &CTRL.pi_iTs, sizeof(sPi_Iq));

    printf("Kp_cur=%g, Ki_cur=%g\n", CTRL.pi_iMs.Kp, CTRL.pi_iMs.Ki);
    CTRL.pi_HFI.Kp = 0;
    //150;
    CTRL.pi_HFI.Ti = 0.08;
    CTRL.pi_HFI.Ki = 1.6;
    CTRL.pi_HFI.i_state = 0.0;
    CTRL.pi_HFI.i_limit = 2 * M_PI;
    // 28*M_PI / 180; // unit: Volt, 350V->max 1300rpm
}

float tmpold = 0;
float tmp, isdlowold = 0;
float isdxold[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float isdyold[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float isqxold[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float isqyold[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#define REF_RAMP_STEP_OPENLOOP 0.0001
double speedref = 0;
double rotortheta0 = -M_PI / 2;

void abtodq(double a, double b, double theta, double *d, double *q)
{
    double cosT = cos(theta);
    double sinT = sin(theta);

    *d = AB2M(a, b, cosT, sinT);
    *q = AB2T(a, b, cosT, sinT);
}

void dqtoab(double d, double q, double theta, double *a, double *b)
{
    double cosT = cos(theta);
    double sinT = sin(theta);

    *a = MT2A(d, q, cosT, sinT);
    *b = MT2B(d, q, cosT, sinT);
}

void openloop_control(double speed_cmd, double speed_cmd_dot, double runtine)
{
    speedref += REF_RAMP_STEP_OPENLOOP;
    double iq0ref = 2.0;
    double id0ref = 0;
    double wref = speedref * 2 * M_PI;

    // if (runtine > 1)
    {
        rotortheta0 += wref * TS;
        rotortheta0 = rounddegree(rotortheta0);
        // rotortheta0 = -M_PI / 2;
    }

    dbg_tst(15, rotortheta0);

    double ia = sm.is_curr[0];
    double ib = sm.is_curr[1];

    double id, iq;
    abtodq(ia, ib, rotortheta0, &id, &iq);
    dbg_tst(13, id);
    dbg_tst(14, iq);

    double ud, uq;
    ud = -PI(&sPi_Id, id - id0ref);
    uq = -PI(&sPi_Iq, iq - iq0ref);

    dbg_tst(11, ud);
    dbg_tst(12, uq);

    dqtoab(ud, uq, rotortheta0, &CTRL.ual, &CTRL.ube);
}
static double theta_d_harnefors = 0.0;
static double omg_harnefors = 0.0;

double sign(double x)
{
    if(x >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}
#define L_COEFF 1//0.8
#define R_COEFF 1//1.03
void harnefors_scvm()
{
#define KE_MISMATCH 1.0 // 0.7
    double d_axis_emf;
    double q_axis_emf;
#define LAMBDA 2        // 2
#define CJH_TUNING_A 1  // 1
#define CJH_TUNING_B 1  // 1
    double lambda_s = LAMBDA * sign(omg_harnefors);
    double alpha_bw_lpf = CJH_TUNING_A * 0.1 * (1000 * RPM_2_RAD_PER_SEC(ACM.npp)) + CJH_TUNING_B * 2 * LAMBDA * fabs(omg_harnefors);
    // d_axis_emf = CTRL.ud_cmd - 1*CTRL.R*CTRL.id_cmd + omg_harnefors*1.0*CTRL.Lq*CTRL.iq_cmd; // If Ld=Lq.
    // q_axis_emf = CTRL.uq_cmd - 1*CTRL.R*CTRL.iq_cmd - omg_harnefors*1.0*CTRL.Ld*CTRL.id_cmd; // If Ld=Lq.
    d_axis_emf = CTRL.uMs_cmd - 1 * CTRL.R * R_COEFF * CTRL.iMs_cmd + omg_harnefors * 1.0 * L_COEFF * CTRL.Lq * CTRL.iTs_cmd; // eemf
    q_axis_emf = CTRL.uTs_cmd - 1 * CTRL.R * R_COEFF * CTRL.iTs_cmd - omg_harnefors * 1.0 * CTRL.Lq * CTRL.iMs_cmd;       // eemf
    // Note it is bad habit to write numerical integration explictly like this. The states on the right may be accencidentally modified on the run.
    omg_harnefors += TS * alpha_bw_lpf * ((q_axis_emf - lambda_s * d_axis_emf) / (CTRL.KE * KE_MISMATCH + (CTRL.Ld - CTRL.Lq) * CTRL.iMs_cmd) - omg_harnefors);
    theta_d_harnefors += TS * omg_harnefors;

    if (theta_d_harnefors > M_PI)
    {
        theta_d_harnefors -= 2 * M_PI;
    }
    if (theta_d_harnefors < -M_PI)
    {
        theta_d_harnefors += 2 * M_PI;
    }
    dbg_tst(29, omg_harnefors * RAD_PER_SEC_2_RPM(ACM.npp));
    dbg_tst(28, theta_d_harnefors);
    dbg_tst(18, d_axis_emf);
    dbg_tst(19, q_axis_emf);
}

void control(double speed_cmd, double speed_cmd_dot)
{
    // Input 1 is feedback: estimated speed or measured speed
    dbg_tst(15, CTRL.theta_M);

    // Input 2 is feedback: measured current
    CTRL.ial_fb = sm.is_curr[0];
    CTRL.ibe_fb = sm.is_curr[1];
    // Input 3 is the rotor d-axis position
#if SENSORLESS_CONTROL
    // getch("Not Implemented");
#else

    //  +param *M_PI / 180.0f;
    #if ANGLE_DETECTION_HFI == 1
    CTRL.pi_HFI.Ki = (0.8 + 0.068 * (CTRL.omg_fb)) * 1.5 / 30;
    CTRL.theta_M = PI_Degree(&CTRL.pi_HFI, tmp);
    CTRL.theta_M += M_PI / 2; //0.14875 * CTRL.omg_fb;
    CTRL.theta_M = rounddegree(CTRL.theta_M);
    // CTRL.theta_M = rounddegree(CTRL.theta_M + M_PI);
    #endif
    // CTRL.theta_M = sm.theta_d - 10 * M_PI / 180;
    float xx = rounddegree(CTRL.theta_M);
    // CTRL.theta_M = xx;
    // CTRL.pi_HFI.i_state = xx;
#endif
    CTRL.theta_M = sm.theta_d;
#if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
    // Flux (linkage) command
    CTRL.rotor_flux_cmd = 0.0;
#endif

    // M-axis current command


    // CTRL.iMs_cmd = MAX(CTRL.iMs_cmd, 3);
    // T-axis current command
    static int vc_count = 0;
    if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;
        CTRL.omg_ctrl_err = CTRL.omg_fb - speed_cmd * RPM_2_RAD_PER_SEC(ACM.npp);
        CTRL.iTs_cmd = -PI(&CTRL.pi_speed, CTRL.omg_ctrl_err);

        CTRL.speed_ctrl_err = CTRL.omg_ctrl_err * RAD_PER_SEC_2_RPM(ACM.npp);
    }

    dbg_tst(24, speed_cmd);
    dbg_tst(25, CTRL.omg_fb * RAD_PER_SEC_2_RPM(ACM.npp));
    dbg_tst(27, CTRL.omg_ctrl_err);

    /*     if (fabs(CTRL.omg_fb) > 0.2 * 2000)
    {
        CTRL.iMs_cmd = CTRL.rotor_flux_cmd / CTRL.Ld;
        printf("Over gate!");
    }
    else */
    {
        CTRL.iMs_cmd = 0; //CTRL.iTs_cmd / LAMBDA * sign(CTRL.omg_fb);
    }

#if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
    CTRL.cosT = cos(CTRL.theta_M);
    CTRL.sinT = sin(CTRL.theta_M);
#endif

    // Measured current in M-T frame
    CTRL.iMs = AB2M(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);
    CTRL.iTs = AB2T(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);

#if ANGLE_DETECTION_HFI == 1
 float a = 0.0005;
    float c = filter(CTRL.iTs, isqxold, isqyold);
    float b = c * -1 * sin(theta_hfi);
    tmp = b * a + (1 - a) * tmpold; // Cut frequency cal = a/(2*pi*ts)
    tmpold = tmp;

    float isdband, isdsin, isdlow;

    isdband = filter(CTRL.iMs, isdxold, isdyold);
    isdsin = isdband * -1 * sin(theta_hfi);
    isdlow = CTRL.iMs * a + (1 - a) * isdlowold;
    isdlowold = isdlow;
#endif
    // Voltage command in M-T frame
    double vM, vT;
    vM = -PI(&CTRL.pi_iMs, CTRL.iMs - CTRL.iMs_cmd);
    vT = -PI(&CTRL.pi_iTs, CTRL.iTs - CTRL.iTs_cmd);

    dbg_tst(11, vM);
    dbg_tst(12, vT);
    dbg_tst(13, CTRL.iMs);
    // dbg_tst(14, CTRL.iTs);
    dbg_tst(28, CTRL.iMs_cmd);
    dbg_tst(29, CTRL.iTs_cmd);
#if ANGLE_DETECTION_HFI == 1
    float fHFI = HFI_Voltage(TS);
#else
    float fHFI = 0;
#endif
    // Current loop decoupling (skipped for now)
    CTRL.uMs_cmd = vM + fHFI;
    CTRL.uTs_cmd = vT;
    // Voltage command in alpha-beta frame
    CTRL.ual = MT2A(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);

    /*     CTRL.ud_cmd = CTRL.uMs_cmd;
    CTRL.uq_cmd = CTRL.uTs_cmd;
    CTRL.iq_cmd = CTRL.iTs_cmd;
    CTRL.id_cmd = CTRL.iMs_cmd; */
#if USING_HARNEFORS == 1
    // getch("Not Implemented");
    // CTRL.omg_fb    ;
    // CTRL.omega_syn ;
    harnefors_scvm();
    CTRL.omg_fb = omg_harnefors;
    CTRL.theta_M = theta_d_harnefors;
#else
    CTRL.omg_fb = sm.omg;
    CTRL.theta_M = sm.theta_d;
#endif
}

#endif

void measurement()
{
    sm.us_curr[0] = CTRL.ual;
    sm.us_curr[1] = CTRL.ube;
    sm.is_curr[0] = ACM.ial;
    sm.is_curr[1] = ACM.ibe;
    sm.omg = ACM.omg;
    sm.theta_d = ACM.theta_d;
    sm.theta_r = sm.theta_d;
}