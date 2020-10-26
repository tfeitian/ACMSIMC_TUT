#ifndef DRIVEPH_H
#define DRIVEPH_H
#include "stdint.h"

#define DRIVE_ENABLE 0x10  //正常运行
#define DRIVE_DISABLE 0xF0 //6个IGBT全关
#define DRIVE_BRAKE 0xE1   //上下桥个50%占空比刹车

typedef struct
{
   uint32_t CCR1; /*!< TIM capture/compare register 1,              Address offset: 0x34 */
   uint32_t CCR2; /*!< TIM capture/compare register 2,              Address offset: 0x38 */
   uint32_t CCR3; /*!< TIM capture/compare register 3,              Address offset: 0x3C */
   uint32_t CCR4; /*!< TIM capture/compare register 4,              Address offset: 0x40 */
} TIM_TypeDef;

typedef struct
{
   //inputs
   struct
   {

      u8 ubEnable;     // 0xeabd=enable comm based on cntr,
                       // 0xfabc=forced comm based on drivePhase
                       // 0x0=disable
      u16 uwPwmPeriod; // PWM period
      u16 uwTa;
      u16 uwTb;
      u16 uwTc;

   } inputs;

} DRIVEPH_OBJECT;
#define DRIVE_DEFAULTS                         \
   {                                           \
      DRIVE_DISABLE, 1500, 32767, 32767, 32767 \
   }
void PWM_Initialize(void);
void PWM_Update(DRIVEPH_OBJECT *p);
extern DRIVEPH_OBJECT Driver1;

#define DISABLE_ALL_SWITCH                              \
   SET_ALL_PINS_TO_PWM;                                 \
   Driver1.inputs.ubEnable = DRIVE_DISABLE;             \
   TIM1->BDTR &= (uint16_t)(~((uint16_t)TIM_BDTR_MOE)); \
   ADC1->CR |= ADC_CR_ADSTP;                            \
   MotorStateMachine.ubNeedManuTrigAd = 1

#define ENABLE_ALL_SWITCH                  \
   SET_ALL_PINS_TO_PWM;                    \
   Driver1.inputs.ubEnable = DRIVE_ENABLE; \
   TIM1->BDTR |= TIM_BDTR_MOE

#define ENABLE_BRAKE                      \
   SET_H_PINS_TO_IO;                      \
   SET_H_IOMODE_PINS_TO_OFF;              \
   SET_L_PINS_TO_PWM;                     \
   Driver1.inputs.ubEnable = DRIVE_BRAKE; \
   TIM1->BDTR |= TIM_BDTR_MOE

#endif