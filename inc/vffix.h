#ifndef VF_FIX_H
#define VF_FIX_H
#include "userdefine.h"

void ufinit(void);
float uf_control(double speedref, double noused);
void vffix_control(double speed_cmd, double speed_cmd_dot);
s32 ufcontrol(double speed_cmd, double speed_cmd_dot);
s32 ufcontrol0(double speedref, double noused);
;
#endif // ! VF_FIX_H