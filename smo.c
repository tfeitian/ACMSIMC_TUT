#include <math.h>
#include "ACMSim.h"

#define T_SAMPLE ((float)TS) //Sample time
#define Fre_Max 200          // Motor max frequency
#define Pi 3.1415926
#define MOTOR_POLES 5
#define MIN_SPEED 200
#define MAX_VOLTAGE 300

#define MAX_CURRENT 3

static float fFsmopos, fGsmopos, fKslide, fKslfMin, fKslf, fKOmgKslf, fKCoeff, fMaxCurrentError;

void smo_vInit(float fRs, float fLs)
{
    double tmpx1 = (1 - (float)T_SAMPLE * fRs / fLs);
    fFsmopos = tmpx1;

    tmpx1 = (float)T_SAMPLE / fLs;

    fGsmopos = tmpx1;

    fKslide = 0.3662;

    fKslfMin = (float)MIN_SPEED * (float)MOTOR_POLES * T_SAMPLE;

    fKslf = fKslfMin;

    fKOmgKslf = fKslf;

    fKCoeff = 2.0 * Pi * (float)Fre_Max * T_SAMPLE;

    tmpx1 = 20.0 * (float)MOTOR_POLES * 2 / 60 * T_SAMPLE;

    fMaxCurrentError = 3.3f; // TODO: Need to be transfered into float
}

static float fEstIa, fEstIb, fZa, fZb, fEa, fEb, fOmega, fOmegFiltered, fEaFiltered, fEbFiltered;

float smo_vCalc(float fIa, float fIb, float fUa, float fUb, float fOmega)
{

    fEstIa = fFsmopos * fEstIa + fGsmopos * (fUa - fEa - fZa);
    fEstIb = fFsmopos * fEstIb + fGsmopos * (fUb - fEb - fZb);

    float fIaError = fEstIa - fIa;
    float fIbError = fEstIb - fIb;

    if (fIa >= fMaxCurrentError)
    {
        fZa = fKslide;
    }
    else if (fIa <= -fMaxCurrentError)
    {
        fZa = -fKslide;
    }
    else
    {
        fZa = fKslide * fIaError / fMaxCurrentError;
    }

    if (fIb >= fMaxCurrentError)
    {
        fZb = fKslide;
    }
    else if (fIb <= -fMaxCurrentError)
    {
        fZb = -fKslide;
    }
    else
    {
        fZb = fKslide * fIbError / fMaxCurrentError;
    }

    fOmegFiltered = fOmegFiltered + (fOmega - fOmegFiltered) * fKOmgKslf;

    fKslf = fOmegFiltered * fKCoeff;

    if (fKslf > fKslfMin)
    {
        fKslf = fKslfMin;
    }

    fEa = fEa + (fKslf * (fZa - fEa));
    fEb = fEb + (fKslf * (fZb - fEb));

    fEaFiltered = fEaFiltered + fKslf * (fEa - fEaFiltered);
    fEbFiltered = fEbFiltered + fKslf * (fEb - fEbFiltered);

    float ftheta = atan2f(-fEaFiltered, fEbFiltered);
    return ftheta;
}