#ifndef VF_FIX_H
#define VF_FIX_H
#include "userdefine.h"

void ufinit(void);
void vffix_control(double speed_cmd, double speed_cmd_dot);
s32 ufcontrol(double speed_cmd, double speed_cmd_dot);
#endif // ! VF_FIX_H