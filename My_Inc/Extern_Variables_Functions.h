/**
  ******************************************************************************
  * @file Extern_Variables_functions.h
  * @author SangTN - FPT Company
  * @version V1.1
  * @date 11-20-2018
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EXTERN_VARIABLES_FUNCTIONS_H
#define __EXTERN_VARIABLES_FUNCTIONS_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#ifdef STM32H743xx
  #include "stm32h7xx_hal.h"
  #include "core_cm7.h"
#else
  #include "stm32f4xx_hal.h"
#endif

#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "generic_type_defs.h"
#include "grbl.h"
#include "Config_stm32f4.h"
#include "Delay.h"
#include "Encoder.h"

/* Extern types ------------------------------------------------------------*/
/* Extern constants --------------------------------------------------------*/
/* Extern macro ------------------------------------------------------------*/

//#define TEST_PULSE
#define USER_ENCODER_TEST
#define USER_MANY_PRESCALER
#define USER_CPU_CLOCK_200MHZ
#define USER_ODR

#define COUNT_TIMER

//#define USER_ACTIVE_FLAG
//#define USER_ACTIVE_IT
#define USER_UNLOCK_AFTER_RESET
#define USER_NOT_STEP_PORT_INV
#define USER_SAVE_POSITION
#ifdef USER_BACKLASH
  #define USER_SAVE_BACKLASH
#endif
//#define  USER_ENCODER_FULL_32BIT
//#define  NOT_USED_STEP_PORT_SIMULATE
//#define USER_STEP_PULSE_DUTY_CHANGE
#define USER_DISPLAY_PULSE_LED

// #define BAUD_RATE 230400
#define BAUD_RATE 115200
//#define BAUD_RATE 9600

// When a M2 or M30 program end command is executed, most g-code states are restored to their defaults.
// This compile-time option includes the restoring of the feed, rapid, and spindle speed override values
// to their default values at program end.
//#define RESTORE_OVERRIDES_AFTER_PROGRAM_END // Default enabled. Comment to disable.

// Adaptive Multi-Axis Step Smoothing (AMASS) is an advanced feature that does what its name implies,
// smoothing the stepping of multi-axis motions. This feature smooths motion particularly at low step
// frequencies below 10kHz, where the aliasing between axes of multi-axis motions can cause audible
// noise and shake your machine. At even lower step frequencies, AMASS adapts and provides even better
// step smoothing. See stepper.c for more details on the AMASS system works.

//#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // Default enabled. Comment to disable.

// Sets the maximum step rate allowed to be written as a Grbl setting. This option enables an error
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.
#ifdef STM32H743xx
  #define MAX_STEP_RATE_HZ 400000 // Hz
#else
  #define MAX_STEP_RATE_HZ 250000 // Hz
#endif

  #if defined( USER_CPU_CLOCK_250MHZ )
    #define F_CPU 25000000
    #define F_CPU_60 1500000000
    //#define TICKS_PER_MICROSECOND_DELAY ((F_CPU/2)/100000)//~80 if fcpu=16x10^6
    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)250
  #elif defined( USER_CPU_CLOCK_240MHZ )
    #define F_CPU 24000000
    #define F_CPU_60 1440000000
    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)240
  #elif defined( USER_CPU_CLOCK_216MHZ )
    #define F_CPU 21600000
    #define F_CPU_60 1296000000
    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)216
  #elif defined( USER_CPU_CLOCK_200MHZ )
    #define F_CPU 20000000
    #define F_CPU_60 1200000000

    #ifdef STM32H743xx
      #define MUL_TIMER10 10 //200M(TIMER CLOCK) / F_CPU = 10
    #else
      #define MUL_TIMER10 5 //100M(TIMER CLOCK) / F_CPU = 5
    #endif

    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)200 //200M(TIMER CLOCK)

    #define PWM_PER_100  92
  #elif defined( USER_CPU_CLOCK_180MHZ )
    #define F_CPU 18000000
    #define F_CPU_60 1080000000
    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)180
  #elif defined( USER_CPU_CLOCK_168MHZ )
    #define F_CPU 16800000
    #define F_CPU_60 1008000000
    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)168
  #elif defined( USER_CPU_CLOCK_160MHZ )
    #define F_CPU 16000000
    #define F_CPU_60 960000000
    #define MUL_TIMER10 5 //80M(TIMER CLOCK) / F_CPU = 5
    #define TICKS_PER_MICROSECOND_DELAY (uint16_t)160
  #endif
 // CoreXY motor assignments. DO NOT ALTER.
 // NOTE: If the A and B motor axis bindings are changed, this effects the CoreXY equations.
 #ifdef COREXY
  #define A_MOTOR X_AXIS // Must be X_AXIS
  #define B_MOTOR Y_AXIS // Must be Y_AXIS
 #endif

 extern uint8_t DirectionBitSimulateArray[];
 extern uint8_t StepBitSimulateArray[];
 extern uint8_t LimitBitSimulateArray[];

 #ifdef USER_SAVE_POSITION
   extern uint32_t aBKPDataReg[];
   #if defined ( __GNUC__ )
     extern uint8_t BufferBackup[];
     extern int32_t BufferBackupRTC32[];
     extern uint8_t BufferBackupRTC8[];
   #endif
 #endif
/* Extern variable ------------------------------------------------------- */

  extern RTC_HandleTypeDef hrtc;

  extern TIM_HandleTypeDef htim1;
  extern TIM_HandleTypeDef htim2;
  extern TIM_HandleTypeDef htim3;
  extern TIM_HandleTypeDef htim4;
  extern TIM_HandleTypeDef htim5;
  extern TIM_HandleTypeDef htim6;
  extern TIM_HandleTypeDef htim7;
  extern TIM_HandleTypeDef htim8;
  extern TIM_HandleTypeDef htim13;

  extern UART_HandleTypeDef huart3;
  #ifdef STM32H743xx
  #else
    extern UART_HandleTypeDef huart6;
  #endif


  extern int32_t CountTimer1Enc;
  extern int32_t CountTimer1EncOver;
  extern int32_t CountTimer2Enc;
  extern int32_t CountTimer2EncOver;
  extern int32_t CountTimer3Enc;
  extern int32_t CountTimer3EncOver;
  extern int32_t CountTimer4Enc;
  extern int32_t CountTimer4EncOver;
  extern int32_t CountTimer5Enc;
  extern int32_t CountTimer5EncOver;
  extern int32_t CountTimer8Enc;
  extern int32_t CountTimer8EncOver;

  /* Extern function prototypes ------------------------------------------------------- */
  extern void TIM_CCxNChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelNState);

#ifdef __cplusplus
}
#endif
#endif /*__EXTERN_VARIABLES_FUNCTIONS_H */


/**
  * @}
  */ 

/**
  * @}
  */ 

/*****************************END OF FILE****/
