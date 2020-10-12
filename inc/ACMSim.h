#ifndef ACMSIM_H
#define ACMSIM_H

#define INDUCTION_MACHINE 1
#define SYNCHRONOUS_MACHINE 2
#define MACHINE_TYPE SYNCHRONOUS_MACHINE

/* standard lib */
#include <stdio.h> // printf #include <stdbool.h> // bool for _Bool and true for 1
// #include <stdbool.h> // bool for _Bool and true for 1
#include <process.h> //reqd. for system function prototype
#include <conio.h>   // for clrscr, and getch()
#include "stdlib.h"  // for rand()
#include "math.h"
#include "time.h"
#include "userdefine.h"
#include "config.h"

#if MACHINE_TYPE == INDUCTION_MACHINE
#define VVVF_CONTROL 0
#define IFOC 1
#define DFOC 2
#define CONTROL_STRATEGY IFOC

#define SENSORLESS_CONTROL true

#elif MACHINE_TYPE == SYNCHRONOUS_MACHINE
#define NULL_D_AXIS_CURRENT_CONTROL -1
#define MTPA -2 // not supported
#define CONTROL_STRATEGY NULL_D_AXIS_CURRENT_CONTROL

#endif

/* How long should I go? */
#define NUMBER_OF_LINES (100000)

#define MACHINE_TS (1.25e-4 / 2)
#define MACHINE_TS_INVERSE 8000
#define DOWN_FREQ_EXE 1
#define DOWN_FREQ_EXE_INVERSE (1.0 / DOWN_FREQ_EXE)
#define TS (MACHINE_TS * DOWN_FREQ_EXE)                         //2.5e-4
#define TS_INVERSE (MACHINE_TS_INVERSE * DOWN_FREQ_EXE_INVERSE) // 4000

#define PHASE_NUMBER 3

/* Declaration of Utility Function */
void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
#endif