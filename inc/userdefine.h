#ifndef USER_DEFINIE_H
#define USER_DEFINIE_H
#include <stdint.h>
#include "math.h"
/* Macro for Part Transformation*/
#define AB2M(A, B, COS, SIN) ((A)*COS + (B)*SIN)
#define AB2T(A, B, COS, SIN) ((A) * -SIN + (B)*COS)
#define MT2A(M, T, COS, SIN) ((M)*COS - (T)*SIN)
#define MT2B(M, T, COS, SIN) ((M)*SIN + (T)*COS)

/* General Constants */
#define TRUE True
#define FALSE False
#define True (1)
#define False (0)
#define true 1
#define false 0
#define lTpi 0.15915494309189535 // 1/(2*pi)
#define TWO_PI_OVER_3 2.0943951023931953
#define SIN_2PI_SLASH_3 0.86602540378443871       // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.86602540378443871 // sin(-2*pi/3)
#define SQRT_2_SLASH_3 0.81649658092772603        // sqrt(2.0/3.0)
// #define RPM_2_RAD_PER_SEC     0.20943951023931953 // (2/60*Tpi)
// #define RAD_PER_SEC_2_RPM     4.7746482927568605 // 1/(im.npp/60*Tpi)
#define abs use_fabs_instead_or_you_will_regret
#define RAD_PER_SEC_2_RPM(NPP) (60.0 / (2 * M_PI * NPP))
#define RPM_2_RAD_PER_SEC(NPP) ((2 * M_PI * NPP) / 60.0)
// #define PI_D 3.1415926535897932384626433832795 /* double */
#define M_PI_OVER_180 0.017453292519943295

// 补充的宏，为了实现实验/仿真代码大一统
#define Uint32 unsigned long int
#define Uint16 unsigned int

#define MAX(a, b) ((a > b ? a : b))
#define MIN(a, b) ((a > b ? b : a))
#define LIMIT(x, a, b) (MAX(MIN(x, a), b))

typedef uint16_t u16;
typedef uint32_t u32;
typedef int16_t s16;
typedef int32_t s32;
typedef uint8_t u8;
typedef long long s64;
typedef unsigned long long u64;
// typedef _Bool bool;

#define CONTROL_METHOD VF_CONTROL
#endif // !USER_DEFINIE_H
