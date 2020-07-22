#include <math.h>
#include "ACMSim.h"
#include "tool.h"

#define T_SAMPLE ((float)TS) //Sample time
#define Fre_Max 200          // Motor max frequency
#define Pi 3.1415926
#define MOTOR_POLES 5
#define MIN_SPEED 200
#define MAX_VOLTAGE 300

#define MAX_CURRENT 3

static float fFsmopos, fGsmopos, fKslide, fKslfMin, fKslf, fKOmgKslf, fKCoeff, fMaxCurrentError;
float fGsmoposTmp = 0;
void smo_vInit(float fRs, float fLs)
{
    double tmpx1 = (1 - (float)T_SAMPLE * fRs / fLs);
    tmpx1 = exp(-fRs / fLs * (float)T_SAMPLE);
    fFsmopos = tmpx1;

    tmpx1 = (float)T_SAMPLE / (fLs);

    fGsmopos = tmpx1;
    fGsmoposTmp = 1 / fRs * (1 - fFsmopos);
    fKslide = 0.3662;

    fKslfMin = (float)MIN_SPEED * (float)MOTOR_POLES * T_SAMPLE;

    fKslf = fKslfMin;

    fKOmgKslf = fKslf;

    fKCoeff = 2.0 * Pi * (float)Fre_Max * T_SAMPLE;

    tmpx1 = 20.0 * (float)MOTOR_POLES * 2 / 60 * T_SAMPLE;

    fMaxCurrentError = 1.5;
    //105.3f / 200; // !This value is the key parameter for different load

    fKslf = 0.1057;
    fKslide = 0.25;
}

static float fEstIa = 0.0f, fEstIb = 0.0f, fZa = 0.0f, fZb = 0.0f, fEa = 0.0f, fEb = 0.0f, fOmega, fOmegFiltered, fEaFiltered, fEbFiltered;

float smo_vCalc(float fIa, float fIb, float fUa, float fUb, float fOmega)
{
    fGsmopos = fGsmoposTmp; // * (245.37f / 2.5f);

    fEstIa = fFsmopos * fEstIa + fGsmopos * (fUa - fEa - fZa);
    fEstIb = fFsmopos * fEstIb + fGsmopos * (fUb - fEb - fZb);

    float fIaError = fEstIa - fIa;
    float fIbError = fEstIb - fIb;

    if (fIaError >= fMaxCurrentError)
    {
        fZa = fKslide;
    }
    else if (fIaError <= -fMaxCurrentError)
    {
        fZa = -fKslide;
    }
    else
    {
        fZa = fKslide * fIaError / fMaxCurrentError;
    }

    if (fIbError >= fMaxCurrentError)
    {
        fZb = fKslide;
    }
    else if (fIbError <= -fMaxCurrentError)
    {
        fZb = -fKslide;
    }
    else
    {
        fZb = fKslide * fIbError / fMaxCurrentError;
    }

    fOmegFiltered = fOmega;
    // fOmegFiltered + (fOmega - fOmegFiltered) * fKOmgKslf;

    fKslf = fOmegFiltered * fKCoeff;

    if (fKslf > fKslfMin)
    {
        fKslf = fKslfMin;
    }
    float ftheta = atan2f(-fZa, fZb);

    fKslf = 0.105707397;

    fEa = fEa + (fKslf * (fZa - fEa));
    fEb = fEb + (fKslf * (fZb - fEb));

    fEaFiltered = fEaFiltered + fKslf * (fEa - fEaFiltered);
    fEbFiltered = fEbFiltered + fKslf * (fEb - fEbFiltered);

    float ftheta0 = atan2f(-fEaFiltered, fEbFiltered);
    dbg_tst(10, fEstIa);
    dbg_tst(11, fEstIb);
    dbg_tst(15, fIa);
    dbg_tst(16, fIb);
    dbg_tst(12, ftheta0); //atan2f(-fEa, fEb));
    dbg_tst(13, fEa);
    dbg_tst(14, fEb);
    dbg_tst(17, fZa);
    dbg_tst(18, fZb);

    return ftheta;
}