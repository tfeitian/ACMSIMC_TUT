#include "ACMSim.h"

void inverter_model(double ual, double ube, double theta, double *pud, double *puq)
{
#if MACHINE_TYPE == INDUCTION_MACHINE
    ACM.ual = ual;
    ACM.ube = ube;
#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
    *pud = AB2M(ual, ube, cos(theta), sin(theta));
    *puq = AB2T(ual, ube, cos(theta), sin(theta));
#endif
}