#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "dbglog.h"
#include "userdefine.h"

#define MAX_DBGLOG_LENGTH 100

char *namesArr[MAX_DBGLOG_LENGTH];
u16 namelength = 0;
float dbgarr[MAX_DBGLOG_LENGTH];

bool bHeaderWriten = false;

static void saveheader(FILE *pf)
{
    for (u16 i = 0; i < namelength - 1; i++)
    {
        fputs(namesArr[i], pf);
        fputs(",", pf);
    }
    fputs(namesArr[namelength - 1], pf);
    fputs("\n", pf);
}

void dbginit(void)
{
    dbglog("uffix-vout", 0);
    dbglog("uffix-wset", 0);
    dbglog("uffix-theta", 0);

    dbglog("smoIalpha", 0);
    dbglog("smoIbeta", 0);
    dbglog("smo-udin", 0);
    dbglog("smo-uqin", 0);
    dbglog("smofix", 0);
    dbglog("uwRotorAngleGlob", 0);
    dbglog("smo-iq", 0);
    dbglog("smo-id", 0);
    dbglog("smo-iqref", 0);
    dbglog("smo-idref", 0);
    dbglog("smo-ualpha", 0);
    dbglog("smo-ubeta", 0);
    dbglog("smo-ud", 0);
    dbglog("smo-uq", 0);
    dbglog("smo-Omeg", 0);

    dbglog("IalphaError", 0);
    dbglog("IbetaError", 0);
    dbglog("EalphaFiltered", 0);
    dbglog("EbetaFiltered", 0);

    dbglog("uffix-phi", 0);
    dbglog("uffix-p", 0);
    dbglog("uffix-q", 0);
    dbglog("uffix-dv", 0);
    dbglog("uffix-dw", 0);
    dbglog("uffix-hfp", 0);
    dbglog("uffix-hfp0", 0);
    dbglog("uffix-wref", 0);
    dbglog("uffix-vcomp", 0);
    dbglog("uffix-ftemp", 0);

    dbglog("calud", 0);
    dbglog("caluq", 0);

    dbglog("uffix-is", 0);
    dbglog("uffix-is_theta", 0);
    dbglog("uffix-thetaPhi", 0);
    dbglog("uffix-iscosphi", 0);
    dbglog("uffix-us", 0);
    dbglog("uffix-us0", 0);

    dbglog("uffix-isfilted", 0);
    dbglog("uffix-qfilter", 0);
}

void dbgsave(FILE *pf)
{
    if (!bHeaderWriten)
    {
        saveheader(pf);
        bHeaderWriten = true;
    }
    for (u16 i = 0; i < namelength - 1; i++)
    {
        fprintf(pf, "%g,", dbgarr[i]);
    }
    fprintf(pf, "%g", dbgarr[namelength - 1]);
    fputs("\n", pf);
}

void dbglog(const char *name, float value)
{
    u16 i = 0;
    bool bfound = false;
    for (i = 0; i < namelength; i++)
    {
        if (strcmp(name, namesArr[i]) == 0)
        {
            bfound = true;
            break;
        }
    }
    if (!bfound)
    {
        char *pstr = (char *)malloc(strlen(name) + 1);
        strcpy(pstr, name);
        namesArr[namelength] = pstr;
        namelength++;
        assert(namelength < MAX_DBGLOG_LENGTH);
    }
    dbgarr[i] = value;
}