#ifndef FIXPOINT_H
#define FIXPOINT_H
#include "userdefine.h"

#define VDC_DIV_COFF 160
#define R_SHUNT 0.05
#define Freq_MAX 200
#define MOTOR_POLES 5
#define MAX_SPEED_RPM 1600

#define MAX_CURRENT_PEAK (float)(6.5)
#define MAX_SPEED_FRQ (MAX_SPEED_RPM * MOTOR_POLES * 32768L) / (60 * Freq_MAX)

// #define FP_CURRENT(x) ((s16)(x * R_SHUNT * 32768));
#define FP_VOLTAGE(x) ((u16)((float)x * 4096 / (VDC_DIV_COFF * 3.3)));
#define FP_SPEED(x) ((long)((float)x * MOTOR_POLES * 32768L) / (60 * Freq_MAX));
#define FP_THETA(x) (u16)((float)x * 32768 / M_PI)

void fix_vControl(double speed_cmd, double speed_cmd_dot);
void fix_vinit(void);
s16 FP_CURRENT(float finput);
#endif // !FIXPOINT_H
