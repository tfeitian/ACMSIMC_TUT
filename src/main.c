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
#include "fixpoint.h"

static float g_fTest[20];
float param[10] = {60, 20, 0.1, 2};

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
    printf("NUMBER_OF_LINES: %d\n\n", NUMBER_OF_LINES);
    for (int i = 0; i < MIN(10, argc - 1); i++)
    {
        param[i] = atof(argv[i + 1]);
    }

    printf("Parameter cnt is %d!\n", argc);
    for (int i = 0; i < (sizeof(param) / sizeof(param[0])); i++)
    {
        printf("%d -- %f\n", i, param[i]);
    }

    write_input(argc, argv);
    /* Initialization */
    Machine_init();
    CTRL_init();
    acm_init();
    ob_init();
    fix_vinit();
    // socket_vinit();
    smo_vInit(ACM.R, ACM.L0);

    FILE *fw;
    fw = fopen("algorithm.dat", "w");
    write_header_to_file(fw);

    /* MAIN LOOP */
    clock_t begin, end;
    double timebase = 0.0f;
    begin = clock();
    int _;       // _ for the outer iteration
    int dfe = 0; // dfe for down frequency execution

    double ud = 0.0, uq = 0.0, rpm_cmd = 0.0;
    for (_ = 0; _ < NUMBER_OF_LINES; ++_)
    {
        /* Command and Load Torque */
        if (timebase > 3.5)
        {
            // rpm_cmd = 80;
            ACM.Tload = 0;
        }
        else if (timebase > 2.7)
        {
            // rpm_cmd = 52;
            ACM.Tload = param[E_LOAD_REF];
        }
        else if (timebase > 1.0)
        {
            rpm_cmd = param[E_SPEED_REF];
            // ACM.Tload = param[E_LOAD_REF];
        }
        else
        {
            double k = 0.01 / 2;
            //ACM.omg *ACM.omg *k;
            if (ACM.Tload < 0.5)
            {
                // ACM.Tload = 0.5;
            }
            else if (ACM.Tload > 10)
            {
                ACM.Tload = 10;
            }
        }
        dbg_tst(16, ACM.Tload);

        /* Simulated ACM */
        if (machine_simulation(ud, uq))
        {
            printf("Break the loop.\n");
            break;
        }

        if (++dfe == DOWN_FREQ_EXE)
        {
            dfe = 0;

            /* Time */
            timebase += TS;

            measurement();

            // observation();

            ob.theta = smo_vCalc(sm.is_curr[0], sm.is_curr[1], CTRL.ual, CTRL.ube, sm.omg);

            write_data_to_file(fw);

            // control(ramp(rpm_cmd, param[E_RAMP_TIME], TS), 0);
            // openloop_control(0, 0, timebase);
            fix_vControl(ramp(rpm_cmd, param[E_RAMP_TIME], TS), 0);
        }

        inverter_model(CTRL.ual, CTRL.ube, ACM.theta_d, &ud, &uq);
    }
    end = clock();
    printf("The simulation in C costs %g sec.\n", (double)(end - begin) / CLOCKS_PER_SEC);
    fclose(fw);
    socket_vClose();
    /* Fade out */
    // system("python ./ACMPlot.py");
    // getch();
    // system("pause");
    // system("exit");
    return 0;
}

/* Utility */
void write_header_to_file(FILE *fw)
{
#if MACHINE_TYPE == INDUCTION_MACHINE
    fprintf(fw, "x0, x1, x2, x3, rpm, uMs_cmd, uTs_cmd, iMs_cmd, iMs, iTs_cmd, iTs, psi_mu_al, tajima_rpm\n");

#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
    fprintf(fw, "x0,x1,x2,x3, uMs_cmd, uTs_cmd, iMs_cmd, iMs, iTs_cmd, iTs, obtheta\n");
#endif
}
void write_data_to_file(FILE *fw)
{
    static int bool_animate_on = false;
    static int j = 0, jj = 0; // j,jj for down sampling

    // if(CTRL.timebase>20)
    {
        if (++j == 1)
        {
            j = 0;
#if MACHINE_TYPE == INDUCTION_MACHINE
            // 数目必须对上，否则ACMAnimate会失效，但是不会影响ACMPlot
            fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
                    ACM.x[0], ACM.x[1], ACM.x[2], ACM.x[3], ACM.rpm,
                    CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.iMs_cmd, CTRL.iMs, CTRL.iTs_cmd, CTRL.iTs,
                    ob.psi_mu_al, ob.tajima.omg * RAD_PER_SEC_2_RPM(ACM.npp));
#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
            fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
                    ACM.id, ACM.iq, ACM.omg * RAD_PER_SEC_2_RPM(ACM.npp), ACM.theta_d, //3
                    CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.iMs_cmd, CTRL.iMs,                //7
                    ACM.Tload, ACM.Tem, g_fTest[0],
                    g_fTest[1], g_fTest[2], g_fTest[3],
                    g_fTest[4], g_fTest[5], g_fTest[6],
                    g_fTest[7], g_fTest[8], g_fTest[9],
                    g_fTest[10], g_fTest[11], g_fTest[12],
                    g_fTest[13], g_fTest[14], g_fTest[15],
                    g_fTest[16], g_fTest[17], g_fTest[18],
                    g_fTest[19], g_fTest[20], g_fTest[21]);
#endif
            /*             dbglog("ACM.x[0]", ACM.x[0]);
            dbglog("ACM.x[1]", ACM.x[1]);
            dbglog("ACM.x[2]", ACM.x[2]);
            dbglog("ACM.x[3]", ACM.x[3]);
            dbglog("CTRL.uMs_cmd", CTRL.uMs_cmd);
            dbglog("CTRL.uTs_cmd", CTRL.uTs_cmd);
            dbglog("CTRL.iMs_cmd", CTRL.iMs_cmd);
            dbglog("CTRL.iMs", CTRL.iMs);
            dbglog("CTRL.iTs_cmd", CTRL.iTs_cmd);
            dbglog("CTRL.iTs", CTRL.iTs);
            dbglog("ob.theta", ob.theta); */
        }
    }
    // socket_vSend();

    if (bool_animate_on == false)
    {
        bool_animate_on = true;
        // printf("Start ACMAnimate\n");
        // system("start python ./ACMAnimate.py");
    }
}

void dbg_tst(int tnum, float fnum)
{
    g_fTest[tnum - 10] = fnum;
}