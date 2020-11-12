#ifndef DBG_LOG_H
#define DBG_LOG_H
#include <stdio.h>

void dbginit(void);
void dbgsave(FILE *pf);
void dbglog(const char *name, float value);
#endif // !DBG_LOG_H
