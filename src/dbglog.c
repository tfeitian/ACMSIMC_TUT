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
        printf(namesArr[i]);
        fputs(namesArr[i], pf);
        fputs(",", pf);
    }
    fputs(namesArr[namelength - 1], pf);
    fputs("\n", pf);
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
        printf("%d--%s\n", strlen(name), name);
        strcpy(pstr, name);
        namesArr[namelength] = pstr;
        namelength++;
        assert(namelength < MAX_DBGLOG_LENGTH);
    }
    dbgarr[i] = value;
}