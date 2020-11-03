#ifndef CONFIG_H
#define CONFIG_H

#include "userdefine.h"
#include "ACMSim.h"

#define ANGLE_DETECTION_HFI 0

#define CONTROL_METHOD VF_CONTROL

#define T_SAMPLE ((float)TS) //Sample time
#define Fre_Max 200          // Motor max frequency
#define Pi 3.1415926
#define MOTOR_POLES 5
#define MIN_SPEED 200
#define MAX_VOLTAGE 300

#define MAX_CURRENT 3

#define RS ((float)6.6)   //13.2ohm
#define LS ((float)0.075) //150mH

#endif // !