#include <stdlib.h>

typedef struct
{
    int index;
    float value;
} S_LOG_ITEM;

#define MAX_LOG_LENGTH 2000

static S_LOG_ITEM g_LogArray[MAX_LOG_LENGTH];
static int tcnts = 0;

void log_vlog(int pind, float pvalue)
{
    if (tcnts < MAX_LOG_LENGTH)
    {
        g_LogArray[tcnts].index = pind;
        g_LogArray[tcnts].value = pvalue;
        tcnts++;
    }
}