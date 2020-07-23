#include "ACMSim.h"
#include "smo.h"
#include "log_socket.h"
#include "controller.h"
#include "observer.h"
#include "motor.h"
#include "inverter.h"
#include "userdefine.h"

static float g_fTest[10];

int main()
{
    printf("NUMBER_OF_LINES: %d\n\n", NUMBER_OF_LINES);

    /* Initialization */
    Machine_init();
    CTRL_init();
    acm_init();
    ob_init();
    socket_vinit();
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

    double ud, uq;
    for (_ = 0; _ < NUMBER_OF_LINES; ++_)
    {
        /* Command and Load Torque */
        if (timebase > 5)
        {
            ACM.rpm_cmd = 250;
        }
        else if (timebase > 2.5)
        {
            ACM.Tload = 5;
        }
        else
        {
            ACM.rpm_cmd = 50;
            ACM.Tload = 1;
        }

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

            ob.theta = smo_vCalc(ACM.ial, ACM.ibe, ACM.ual, ACM.ube, sm.omg);

            write_data_to_file(fw);

            control(ACM.rpm_cmd, 0);
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
            fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
                    ACM.x[0], ACM.x[1], ACM.x[2], ACM.x[3],
                    CTRL.uMs_cmd, CTRL.uTs_cmd, CTRL.iMs_cmd, CTRL.iMs,
                    CTRL.iTs_cmd, CTRL.iTs, g_fTest[0],
                    g_fTest[1], g_fTest[2], g_fTest[3],
                    g_fTest[4], g_fTest[5], g_fTest[6],
                    g_fTest[7], g_fTest[8]);
#endif
            dbglog("ACM.x[0]", ACM.x[0]);
            dbglog("ACM.x[1]", ACM.x[1]);
            dbglog("ACM.x[2]", ACM.x[2]);
            dbglog("ACM.x[3]", ACM.x[3]);
            dbglog("CTRL.uMs_cmd", CTRL.uMs_cmd);
            dbglog("CTRL.uTs_cmd", CTRL.uTs_cmd);
            dbglog("CTRL.iMs_cmd", CTRL.iMs_cmd);
            dbglog("CTRL.iMs", CTRL.iMs);
            dbglog("CTRL.iTs_cmd", CTRL.iTs_cmd);
            dbglog("CTRL.iTs", CTRL.iTs);
            dbglog("ob.theta", ob.theta);
        }
    }
    // socket_vSend();

    if (bool_animate_on == false)
    {
        bool_animate_on = true;
        printf("Start ACMAnimate\n");
        // system("start python ./ACMAnimate.py");
    }
}

void dbg_tst(int tnum, float fnum)
{
    g_fTest[tnum - 10] = fnum;
}