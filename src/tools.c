#include "userdefine.h"
#include "math.h"

int isNumber(double x)
{
    // This looks like it should always be true,
    // but it's false if x is a NaN (1.#QNAN0).
    return (x == x);
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}

/* double HighPassFilter_RC_1order(double *Vi, double *Vi_p, double *Vo_p, double sampleFrq)
{
    double Vo;
    double CutFrq, RC, Coff;

    //high pass filter @cutoff frequency = 0.5 Hz
    CutFrq = 0.5;
    RC = (double)1.0 / 2.0 / M_PI / CutFrq;
    Coff = RC / (RC + 1 / sampleFrq);
    Vo = ((*Vi) - (*Vi_p) + (*Vo_p)) * Coff;

    //update
    *Vo_p = Vo;
    *Vi_p = *Vi;

    return Vo;
} */

float HighPassFilter_RC_1order(float *Vi, float *Vi_p, float *Vo_p, float sampleFrq)
{
    float Vo;
    float CutFrq, RC, Coff;

    //high pass filter @cutoff frequency = 0.5 Hz
    CutFrq = 1.5;
    RC = (float)1.0 / 2.0 / M_PI / CutFrq;
    Coff = RC / (RC + 1 / sampleFrq);
    Vo = ((*Vi) - (*Vi_p) + (*Vo_p)) * Coff;

    //update
    *Vo_p = Vo;
    *Vi_p = *Vi;

    return Vo;
}
double LP_Filter(double fin, double fcoef, double *fold)
{
    double ffilter = fin * fcoef + (1 - fcoef) * (*fold);
    *fold = ffilter;
    return ffilter;
}

float f3to2(float ia, float ib, float *ial, float *ibe)
{
    float ic = 0 - ia - ib;
    *ial = ia;
    *ibe = 1.0 / sqrt(3) * (ia + 2 * ib);
}

float f2to3(float ial, float ibe, float *ia, float *ib, float *ic)
{
    *ia = ial;
    *ib = -1.0 / 2 * ial + sqrt(3) / 2 * ibe;
    *ic = -1.0 / 2 * ial - sqrt(3) / 2 * ibe;
}