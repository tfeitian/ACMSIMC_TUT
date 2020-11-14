#include "FixMath.h"
#include "Svpwm.h"
#include "PI_Adjuster.h"
#include "smo.h"
#include "AngleObserver.h"
#include "Fixpoint.h"
#include "config.h"
#include "dbglog.h"
#include "controller.h"
#include "motor.h"

#define OBSERVER_OUTPUT_ANGLE (AngleObserver.outputs.uwEstTheta + 16384 / 2)

MATRIX_CONVERT CurrentConvert;
IPARK VDQ;
SVGENAB sv1;
PIDREG_OBJECT IdRegulate;
PIDREG_OBJECT IqRegulate;
PIDREG_OBJECT SpeedRegulate;
SMOPOS_OBJECT smo1;
Trig_Components TrigOut;
ANGLE_OBSERVER_OBJECT AngleObserver;
u16 uwRotorAngleGlob = 0;
s16 swCurrentCovCof = 0;

void fixsmo_init(void)
{
    CurrentConvert.pTrig = &TrigOut;
    VDQ.pTrig = &TrigOut;

    PidReg_Intialize(&IdRegulate);
    PidReg_Intialize(&IqRegulate);
    PidReg_Intialize(&SpeedRegulate);

    // IqRegulate.inputs.Kp /= 8;

    /*D axis current */
    IdRegulate.OutMax = 20000;
    IdRegulate.OutMin = 0 - IdRegulate.OutMax;
    /*Q axis current */
    IqRegulate.OutMax = 27000;
    IqRegulate.OutMin = 0 - IqRegulate.OutMax;

    /*Speed colsed loop  */
    SpeedRegulate.inputs.Ref = MAX_SPEED_FRQ;
    SpeedRegulate.inputs.Kp = 512 / 8;
    // / 2 / 2 / 2;
    SpeedRegulate.inputs.Ki = 300;
    SpeedRegulate.inputs.Kc = 32768 / 4;
    SpeedRegulate.OutMax = 3200;
    //ClosedLoopCommand.outputs.uwIqCommand;
    SpeedRegulate.OutMin = 0;

    SMO_Intialize(&smo1);

    AngleObserverInitalize(&AngleObserver);

    swCurrentCovCof = (s16)((float)(3.3 * 0.7071 * 1024) / (R_SHUNT * OP_GAIN));
}

void fixsmo_speedpid(s16 SetpointValue)
{
    SpeedRegulate.inputs.Ref = SetpointValue;
    SpeedRegulate.inputs.Fdb = smo1.swOmegfiltered;
    PidReg_Calculate(&SpeedRegulate);
    // IdRegulate.inputs.Ref = 700;
    //500;
    // IqRegulate.inputs.Ref = 1000;
    IqRegulate.inputs.Ref = SpeedRegulate.outputs.Out;
}

void fixsmo_transfer(void)
{
    float ud, uq, theta;
    theta = FLOAT_THETA(uwRotorAngleGlob);
    ud = AB2M(CTRL.ual, CTRL.ube, cos(theta), sin(theta));
    uq = AB2T(CTRL.ual, CTRL.ube, cos(theta), sin(theta));

    s16 sud, suq;
    Park_Calc(sv1.Ualpha, sv1.Ubeta, &sud, &suq, &VDQ);
    // uwRotorAngleGlob = OBSERVER_OUTPUT_ANGLE;
    SpeedRegulate.inputs.Ref = smo1.swOmegfiltered;
    SpeedRegulate.inputs.Fdb = smo1.swOmegfiltered; //
    SpeedRegulate.Ui = IqRegulate.inputs.Fdb;
    SpeedRegulate.SatErr = 0;
    PidReg_Calculate(&SpeedRegulate);
    IqRegulate.inputs.Ref = SpeedRegulate.outputs.Out;
    IqRegulate.SatErr = 0;
    IqRegulate.Ui = uq / 400 * 32768;
    IqRegulate.Ui = suq;
    //VDQ.inputs.slQs;

    IdRegulate.SatErr = 0;
    IdRegulate.Ui = ud / 400 * 32768;
    IdRegulate.Ui = sud;
    //VDQ.inputs.slDs;
    IdRegulate.inputs.Ref = IdRegulate.inputs.Fdb;
}

void fixsmo_control(s16 swIa, s16 swIb, s16 swVdcFiltered, bool bOutput)
{
    if (IdRegulate.inputs.Ref >= 0)
    {
        IdRegulate.inputs.Ref--;
    }
    CurrentConvert.inputs.swA = swIa;
    CurrentConvert.inputs.swB = swIb;
    ClarkeAndPark_Convert(&CurrentConvert);

    smo1.inputs.swIalpha = CurrentConvert.outputs.swAlpha;
    smo1.inputs.swIbeta = CurrentConvert.outputs.swBelt;

    dbglog("smoIalpha", smo1.inputs.swIalpha);
    dbglog("smoIbeta", smo1.inputs.swIbeta);

    float ud, uq, theta;
    theta = FLOAT_THETA(uwRotorAngleGlob);
    ud = AB2M(CTRL.ual, CTRL.ube, cos(theta), sin(theta));
    uq = AB2T(CTRL.ual, CTRL.ube, cos(theta), sin(theta));

    dbglog("smo-udin", ud);
    dbglog("smo-uqin", uq);

    // sv1.Ualpha = (CTRL.ual);
    // sv1.Ubeta = (CTRL.ube);

    smo1.inputs.swValpha = CTRL.ual * 32768 / swVdcFiltered; //ku_scale = 1/UDC*32768
    smo1.inputs.swVbeta = CTRL.ube * 32768 / swVdcFiltered;

    // uwRotorAngleGlob = AngleObserver.outputs.uwEstTheta - 16384;
    // uwRotorAngleGlob = FP_THETA(ACM.theta_d);
    uwRotorAngleGlob = AngleObserver.outputs.uwEstTheta + 16384 / 2; //+uwAngleOffset;
    if (smo1.swKslide < 12000)
    {
        smo1.swKslide++;
    }

    SMOpos_calc(&smo1);

    AngleObserver.inputs.swVsin = -smo1.swEalpha;
    AngleObserver.inputs.swVcos = smo1.swEbeta;

    AngleObserverCalculate(&AngleObserver);
    smo1.swOmeg = AngleObserver.swOmeg;
    dbglog("smofix", smo1.swOmeg);
    dbglog("uwRotorAngleGlob", FLOAT_THETA(uwRotorAngleGlob));

    TrigOut.uwAngle = uwRotorAngleGlob;
    Trig_Functions(&TrigOut);

    IdRegulate.inputs.Fdb = CurrentConvert.outputs.swDs;
    IqRegulate.inputs.Fdb = CurrentConvert.outputs.swQs;

    dbglog("smo-iq", IqRegulate.inputs.Fdb);
    dbglog("smo-id", IdRegulate.inputs.Fdb);
    dbglog("smo-iqref", IqRegulate.inputs.Ref);
    dbglog("smo-idref", IdRegulate.inputs.Ref);
    /*     Current closed-loop control                     */
    PidReg_Calculate(&IdRegulate);
    PidReg_Calculate(&IqRegulate);

    VDQ.inputs.slDs = IdRegulate.outputs.Out;
    VDQ.inputs.slQs = IqRegulate.outputs.Out;

    InvPark_Calc(&VDQ);

    if (bOutput)
    {
        sv1.Ualpha = VDQ.outputs.slAlpha;
        sv1.Ubeta = VDQ.outputs.slBelt;
    }
    dbglog("smo-ualpha", sv1.Ualpha);
    dbglog("smo-ubeta", sv1.Ubeta);

    dbglog("smo-ud", VDQ.inputs.slDs);
    dbglog("smo-uq", VDQ.inputs.slQs);

    Svpwm_Alpha_Belt_Calc(&sv1);

    if (bOutput)
    {
        CTRL.ual = (float)(sv1.Ualpha) / 32768 * swVdcFiltered;
        CTRL.ube = (float)(sv1.Ubeta) / 32768 * swVdcFiltered;
    }

        /*     Driver1.inputs.uwTa = sv1.Ta;
    Driver1.inputs.uwTb = sv1.Tb;
    Driver1.inputs.uwTc = sv1.Tc;
    PWM_Update(&Driver1); */
    }