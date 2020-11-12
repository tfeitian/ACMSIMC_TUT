#ifndef FIX_SMO_H
#define FIX_SMO_H

#include "userdefine.h"

void fixsmo_init(void);

void fixsmo_control(s16 swIa, s16 swIb, s16 swVdcFiltered, bool bOutput);
#endif // !FIX_SMO_H