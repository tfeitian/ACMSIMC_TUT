#ifndef TOOL_H
#define TOOL_H

extern float param[];

typedef enum
{
    E_SPEED_REF = 0,
    E_MOTOR_ANGLE,
    E_LOAD_REF,
} E_INPUT_PARAMETERS;

void dbg_tst(int tnum, float fnum);
#endif // !TOOL_H
