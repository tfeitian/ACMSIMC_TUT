#ifndef VF_CONTROL_H
#define VF_CONTROL_H
#include "userdefine.h"

u8 pre_run(double speed_cmd, double speed_cmd_dot);
void vf_control(double speed_cmd, double speed_cmd_dot);

#endif // !VF_CONTROL_H