#include <math.h>
#include "ACMSim.h"
#include "tools.h"
#include "motor.h"

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

    fMaxCurrentError = 0.03;
    //1.5;
    //105.3f / 200; // !This value is the key parameter for different load

    fKslf = 0.1057;
    fKslide = 53.25; //This value should be larger enough to make sure the SMO stable.
}

static float fEstIa = 0.0f, fEstIb = 0.0f, fZa = 0.0f, fZb = 0.0f, fEa = 0.0f, fEb = 0.0f, fOmega, fOmegFiltered, fEaFiltered, fEbFiltered;

float smo_vCalc(float fIa, float fIb, float fUa, float fUb, float fOmega)
{
    fGsmopos = fGsmoposTmp; // * (245.37f / 2.5f);
                            /*     fEa = ACM.Ea;
    fEb = ACM.Eb;
    fZa = 0;
    fZb = 0; */
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

    return ftheta;
}