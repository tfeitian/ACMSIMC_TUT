#include "ACMSim.h"

void inverter_model(double ual, double ube, double theta, double *pud, double *puq)
{
    *pud = AB2M(ual, ube, cos(theta), sin(theta));
    *puq = AB2T(ual, ube, cos(theta), sin(theta));
}