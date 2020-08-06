
#include "userdefine.h"

static float g_ref = 0;
static float fstep = 0;
float ramp_init(float fref, float ramptime, float timestep)
{
    fstep = fref * timestep / ramptime;
}
float ramp(float fref, float ramptime, float timestep)
{
    float fstep = fref * timestep / ramptime;
    g_ref += fstep;

    return MIN(g_ref, fref);
}