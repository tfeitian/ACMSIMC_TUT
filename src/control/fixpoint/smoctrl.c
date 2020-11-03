#include "FixMath.h"
#include "Svpwm.h"
#include "PI_Adjuster.h"
#include "smo.h"
#include "AngleObserver.h"

MATRIX_CONVERT CurrentConvert;
IPARK VDQ;
SVGENAB sv1;
PIDREG_OBJECT IdRegulate;
PIDREG_OBJECT IqRegulate;
SMOPOS_OBJECT smo1;
Trig_Components TrigOut;
ANGLE_OBSERVER_OBJECT AngleObserver;
u16 uwRotorAngleGlob = 0;

void fixsmo_init(void)
{
}

void fixsmo_control(s16 swIa, s16 swIb, s16 swVdcFiltered)
{
    CurrentConvert.inputs.swA = swIa * 8;
    CurrentConvert.inputs.swB = swIb * 8;
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