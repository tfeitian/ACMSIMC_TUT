#include "FixMath.h"
#include "Svpwm.h"
#include "PI_Adjuster.h"
#include "smo.h"
#include "AngleObserver.h"
#include "Fixpoint.h"
#include "config.h"

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

    /*D axis current */
    IdRegulate.OutMax = 20000;
    IdRegulate.OutMin = 0 - IdRegulate.OutMax;
    /*Q axis current */
    IqRegulate.OutMax = 27000;
    IqRegulate.OutMin = 0 - IqRegulate.OutMax;

    /*Speed colsed loop  */
    SpeedRegulate.inputs.Ref = MAX_SPEED_FRQ;
    SpeedRegulate.inputs.Kp = 512;
    SpeedRegulate.inputs.Ki = 300;
    SpeedRegulate.inputs.Kc = 32768 / 4;
    // SpeedRegulate.OutMax = ClosedLoopCommand.outputs.uwIqCommand;
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
    IdRegulate.inputs.Ref = 0;
    IqRegulate.inputs.Ref = SpeedRegulate.outputs.Out;
}

void fixsmo_control(s16 swIa, s16 swIb, s16 swVdcFiltered)
{
    CurrentConvert.inputs.swA = swIa;
    CurrentConvert.inputs.swB = swIb;
    ClarkeAndPark_Convert(&CurrentConvert);

    smo1.inputs.swIalpha = CurrentConvert.outputs.swAlpha;
    smo1.inputs.swIbeta = CurrentConvert.outputs.swBelt;

    smo1.inputs.swValpha = sv1.Ualpha * swVdcFiltered / 4096;
    smo1.inputs.swVbeta = sv1.Ubeta * swVdcFiltered / 4096;

    uwRotorAngleGlob = AngleObserver.outputs.uwEstTheta + 16384 / 2; //+uwAngleOffset;
    if (smo1.swKslide < 12000)
        smo1.swKslide++;

    SMOpos_calc(&smo1);

    AngleObserver.inputs.swVsin = -smo1.swEalpha;
    AngleObserver.inputs.swVcos = smo1.swEbeta;

    AngleObserverCalculate(&AngleObserver);
    smo1.swOmeg = AngleObserver.swOmeg;

    TrigOut.uwAngle = uwRotorAngleGlob;
    Trig_Functions(&TrigOut);

    IdRegulate.inputs.Fdb = CurrentConvert.outputs.swDs;
    IqRegulate.inputs.Fdb = CurrentConvert.outputs.swQs;

    /*     Current closed-loop control                     */
    PidReg_Calculate(&IdRegulate);
    PidReg_Calculate(&IqRegulate);

    VDQ.inputs.slDs = IdRegulate.outputs.Out;
    VDQ.inputs.slQs = IqRegulate.outputs.Out;

    InvPark_Calc(&VDQ);

    Svpwm_Alpha_Belt_Calc(&sv1);

    /*     Driver1.inputs.uwTa = sv1.Ta;
    Driver1.inputs.uwTb = sv1.Tb;
    Driver1.inputs.uwTc = sv1.Tc;
    PWM_Update(&Driver1); */
}