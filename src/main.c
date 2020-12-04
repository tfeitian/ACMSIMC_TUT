#include <assert.h>
#include "ACMSim.h"
#include "smo.h"
#include "log_socket.h"
#include "controller.h"
#include "observer.h"
#include "motor.h"
#include "inverter.h"
#include "userdefine.h"
#include "ramp.h"
#include "tools.h"
#include "VFControl.h"
#include "vffix.h"
#include "Svpwm.h"
#include "Fixpoint.h"
#include "fixsmo.h"
#include "dbglog.h"

#define MAX_LOG_CNTS 20
static float g_fTest[MAX_LOG_CNTS];
float param[10] = {100, 0, 0, 2, NUMBER_OF_LINES};

void motor_log(void);

void write_input(int argc, char *argv[])
{
    FILE *fw;
    fw = fopen("input.dat", "w");
    for (int i = 1; i < argc; i++)
    {
        fprintf(fw, "%f ", atof(argv[i]));
    }
    fclose(fw);
}

int main(int argc, char *argv[])
{
    for (int i = 0; i < MIN(10, argc - 1); i++)
    {
        param[i] = atof(argv[i + 1]);
    }
    printf("NUMBER_OF_LINES: %d\n\n", param[E_RUN_TIME]);

    printf("Parameter cnt is %d!\n", argc);
    for (int i = 0; i < (sizeof(param) / sizeof(param[0])); i++)
    {
        printf("%d -- %f\n", i, param[i]);
    }
    /* Initialization */
    Machine_init();
    CTRL_init();
    acm_init();
    ob_init();
    fix_vinit();
    // socket_vinit();
    smo_vInit(ACM.R, ACM.L0);
    fixsmo_init();

    FILE *fw;
    fw = fopen("algorithm.dat", "w+");
    // write_header_to_file(fw);

    /* MAIN LOOP */
    clock_t begin, end;
    double timebase = 0.0f;
    begin = clock();
    int sim_step; // _ for the outer iteration
    int dfe = 0; // dfe for down frequency execution
    float fref;
    dbginit();
    ufinit();
    E_RUN_STATE eState = E_RUN_UF;
    E_MOTOR_STATE eMotorState = E_MOTOR_STOP;
    bool bSimOver = false;

    double ud = 0.0, uq = 0.0, rpm_cmd = 0.0;
    for (sim_step = 0; sim_step < param[E_RUN_TIME]; sim_step++)
    {
        if (bSimOver)
        {
            printf("sim_step %d\n", sim_step);
            break;
        }

        if (sim_step > 370000)
        {
            // rpm_cmd = 0;
        }
        else if (sim_step > 205000)
        {
            rpm_cmd = param[E_SPEED_REF];
        }
        else if (sim_step > 170000)
        {
            // ACM.Tload = 0;
            // rpm_cmd = 0;
        }
        else if (sim_step > 70000)
        {
            // ACM.Tload = 0;
            rpm_cmd = param[E_SPEED_REF];
        }
        else if (sim_step > 5000)
        {
            rpm_cmd = param[E_SPEED_REF];
        }

        if (ACM.omg > 10)
        {
            ACM.Tload = param[E_LOAD_REF];

            float nrpm = ACM.omg * 60 / 2 / M_PI;
            const float K_power_n = 2.24E-07;

            // ACM.Tload = nrpm * nrpm * nrpm * K_power_n * 9.5 / nrpm; //, param[E_LOAD_REF]);
        }
        dbglog("CTRL.ud", ud);
        dbglog("CTRL.uq", uq);

        switch (eMotorState)
        {
        case E_MOTOR_STOP:
            if (rpm_cmd != 0)
            {
                eMotorState = E_MOTOR_RUNNING;
            }
            break;

        case E_MOTOR_RUNNING:
            if (rpm_cmd == 0)
            {
                eMotorState = E_MOTOR_STOP;
                ud = 0;
                uq = 0;
                CTRL.ual = 0.0;
                CTRL.ube = 0;
                ramp_set(0);
                eState = E_RUN_UF;
                ufinit();
            }
            break;

        default:
            break;
        }

        /* Simulated ACM */
        if (machine_simulation(ud, uq))
        {
            printf("Break the loop.\n");
            break;
        }
        motor_log();

        if (++dfe == DOWN_FREQ_EXE)
        {
            dfe = 0;

            /* Time */
            timebase += TS;

            measurement();

            // observation();

            ob.theta = smo_vCalc(sm.is_curr[0], sm.is_curr[1], CTRL.ual, CTRL.ube, sm.omg);
            // write_data_to_file(fw);

#if CONTROL_METHOD == VF_CONTROL
            {
                float ia, ib, ic;
                f2to3(ACM.ial, ACM.ibe, &ia, &ib, &ic);
                // ic = 0 - ia - ib;
                dbglog("ACM.ial", ACM.ial);
                dbglog("ACM.ibe", ACM.ibe);
                dbglog("ACM.ia", ia);
                dbglog("ACM.ib", ib);
                dbglog("ACM.ic", ic);

                static u16 smoruncnts = 0;
                static u16 switchtime = 0;

                //Run speed loop per 1ms

                if (sim_step % (50 * 16) == 0)
                {
                    fref = ramp(rpm_cmd, param[E_RAMP_TIME], 0.05);
                }

                s32 wuf = 0;
                if ((sim_step % 16) == 0)
                {
                    fixsmo_speedpid(FP_SPEED2RAD(fref) / 2 / M_PI);
                }
                if (eMotorState == E_MOTOR_RUNNING)
                {
                    switch (eState)
                    {
                    case E_RUN_UF:
                        wuf = uf_control(fref, 0);
                        fixsmo_control(FP_CURRENT(ia), FP_CURRENT(ib), (400), false);
                        // fixsmo_transfer();
                        if (wuf > 30000)
                        {
                            // eState = E_RUN_SWITCHING;

                            CTRL.pi_speed.i_state = 0;
                            CTRL.pi_iMs.i_state = CTRL.iMs;
                            CTRL.pi_iTs.i_state = CTRL.iTs;
                            // eState = E_RUN_FP_SMO;
                        }
                        caludq();
                        switchtime = 0;
                        break;

                    case E_RUN_SWITCHING:
                        uf_control(fref, 0);
                        fixsmo_control(FP_CURRENT(ia), FP_CURRENT(ib), (400), false);
                        switchtime++;
                        if (switchtime >= 0)
                        {
                            eState = E_RUN_FIX_SMO;
                            smoruncnts = 0;
                            fixsmo_transfer();
                            printf("switch -- %d\n", sim_step);
                        }
                        caludq();
                        break;

                    case E_RUN_FIX_SMO:
                        fixsmo_control(FP_CURRENT(ia), FP_CURRENT(ib), (400), true);
                        smoruncnts++;
                        if (smoruncnts > 1800)
                        {
                            // bSimOver = true;
                        }

                        caludq();
                        break;

                    case E_RUN_FP_SMO:
                        control(ramp(rpm_cmd, param[E_RAMP_TIME], TS), 0);
                        break;

                    default:
                        break;
                    }
                }
            }
#elif CONTROL_METHOD == FLOAT_CONTROL
            control(ramp(rpm_cmd, param[E_RAMP_TIME], TS), 0);
#elif CONTROL_METHOD == FIX_CONTROL
            fix_vControl(ramp(rpm_cmd, param[E_RAMP_TIME], TS), 0);
#endif
        }

        inverter_model(CTRL.ual, CTRL.ube, ACM.theta_d, &ud, &uq);
        // dbglog("ob_theta", ob.theta);
        dbglog("fref", fref);
        dbglog("RunState", eState);

        if (param[E_SPEED_REF] > 250 && eState < E_RUN_SWITCHING)
        {
            // continue;
        }
        dbgsave(fw);
    }

    end = clock();
    printf("The simulation in C costs %g sec with %d counts.\n", (double)(end - begin) / CLOCKS_PER_SEC, sim_step);
    fclose(fw);

    //Move to here for informing server.js to load new sim data
    write_input(argc, argv);
    return 0;
}

void motor_log(void)
{
    dbglog("ACM.id", ACM.id);
    dbglog("ACM.iq", ACM.iq);
    dbglog("ACM.omg", ACM.omg);
    dbglog("ACM.theta_d", ACM.theta_d);
    dbglog("ACM.ud", ACM.ud);
    dbglog("ACM.uq", ACM.uq);
    dbglog("ACM.phid", ACM.phid);
    dbglog("ACM.phiq", ACM.phiq);
    dbglog("ACM.Tload", ACM.Tload);
    dbglog("ACM.Tem", ACM.Tem);
}

void dbg_tst(int tnum, float fnum)
{
    assert(tnum - 10 < MAX_LOG_CNTS);
    g_fTest[tnum - 10] = fnum;
}