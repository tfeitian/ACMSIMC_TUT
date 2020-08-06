#include "controller.h"
#include "ACMSim.h"
#include "math.h"
#include "motor.h"
#include "observer.h"
#include "tools.h"

/* PI Control
 * */
static double PI(struct PI_Reg *r, double err)
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
static float Vinj = 100;
static float whfi = 800 * 2 * M_PI;
static float theta_hfi = 0;

static float HFI_Voltage(float dtime)
{
    theta_hfi += whfi * dtime;

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
    CTRL.pi_HFI.Kp = 500;
    CTRL.pi_HFI.Ti = 0.08;
    CTRL.pi_HFI.Ki = 0.8;
    CTRL.pi_HFI.i_state = 0.0;
    CTRL.pi_HFI.i_limit = 2 * M_PI;
    // 28*M_PI / 180; // unit: Volt, 350V->max 1300rpm
}

float tmpold = 0;
float tmp;

void control(double speed_cmd, double speed_cmd_dot)
{
// Input 1 is feedback: estimated speed or measured speed
#if SENSORLESS_CONTROL
    getch("Not Implemented");
    // CTRL.omg_fb    ;
    // CTRL.omega_syn ;
#else
    CTRL.omg_fb = sm.omg;
#endif
    // Input 2 is feedback: measured current
    CTRL.ial_fb = sm.is_curr[0];
    CTRL.ibe_fb = sm.is_curr[1];
// Input 3 is the rotor d-axis position
#if SENSORLESS_CONTROL
    getch("Not Implemented");
#else
    // CTRL.theta_M = sm.theta_d + param * M_PI / 180.0f;
    CTRL.theta_M = PI_Degree(&CTRL.pi_HFI, tmp);
    dbg_tst(29, CTRL.theta_M);
    float xx = rounddegree(CTRL.theta_M);
    // CTRL.theta_M = xx;
    // CTRL.pi_HFI.i_state = xx;
    dbg_tst(10, xx);
#endif

#if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
    // Flux (linkage) command
    CTRL.rotor_flux_cmd = 0.0;
#endif

    // M-axis current command
    CTRL.iMs_cmd = CTRL.rotor_flux_cmd / CTRL.Ld;

    // T-axis current command
    static int vc_count = 0;
    // if (vc_count++ == VC_LOOP_CEILING * DOWN_FREQ_EXE_INVERSE)
    {
        vc_count = 0;
        CTRL.omg_ctrl_err = CTRL.omg_fb - speed_cmd * RPM_2_RAD_PER_SEC(ACM.npp);
        CTRL.iTs_cmd = -PI(&CTRL.pi_speed, CTRL.omg_ctrl_err);

        CTRL.speed_ctrl_err = CTRL.omg_ctrl_err * RAD_PER_SEC_2_RPM(ACM.npp);
    }

#if CONTROL_STRATEGY == NULL_D_AXIS_CURRENT_CONTROL
    CTRL.cosT = cos(CTRL.theta_M);
    CTRL.sinT = sin(CTRL.theta_M);
#endif

    // Measured current in M-T frame
    CTRL.iMs = AB2M(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);
    CTRL.iTs = AB2T(CTRL.ial_fb, CTRL.ibe_fb, CTRL.cosT, CTRL.sinT);

    float a = 0.0028;
    float c = observation(CTRL.iTs);
    float b = c * -1 * sin(theta_hfi);
    tmp = b * a + (1 - a) * tmpold; // Cut frequency cal = a/(2*pi*ts)
    tmpold = tmp;
    dbg_tst(25, tmp);
    dbg_tst(26, b);
    dbg_tst(27, c);
    dbg_tst(28, CTRL.iTs);
    // Voltage command in M-T frame
    double vM, vT;
    vM = -PI(&CTRL.pi_iMs, CTRL.iMs - CTRL.iMs_cmd);
    vT = -PI(&CTRL.pi_iTs, CTRL.iTs - CTRL.iTs_cmd);

    // Current loop decoupling (skipped for now)
    CTRL.uMs_cmd = vM + HFI_Voltage(TS);
    CTRL.uTs_cmd = vT;

    // Voltage command in alpha-beta frame
    CTRL.ual = MT2A(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
    CTRL.ube = MT2B(CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.cosT, CTRL.sinT);
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