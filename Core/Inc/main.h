/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx.h"
#include "stm32h7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#ifdef STM32H743xx
  #include "stm32h7xx_hal.h"
#else
  #include "stm32f4xx_hal.h"
#endif
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_RESET_GRBL_Pin LL_GPIO_PIN_13
#define B1_RESET_GRBL_GPIO_Port GPIOC
#define B1_RESET_GRBL_EXTI_IRQn EXTI15_10_IRQn
#define X_STEP_Pin LL_GPIO_PIN_0
#define X_STEP_GPIO_Port GPIOF
#define Y_STEP_Pin LL_GPIO_PIN_1
#define Y_STEP_GPIO_Port GPIOF
#define Z_STEP_Pin LL_GPIO_PIN_2
#define Z_STEP_GPIO_Port GPIOF
#define A_STEP_Pin LL_GPIO_PIN_3
#define A_STEP_GPIO_Port GPIOF
#define B_STEP_Pin LL_GPIO_PIN_4
#define B_STEP_GPIO_Port GPIOF
#define C_STEP_Pin LL_GPIO_PIN_5
#define C_STEP_GPIO_Port GPIOF
#define D_STEP_Pin LL_GPIO_PIN_6
#define D_STEP_GPIO_Port GPIOF
#define E_STEP_Pin LL_GPIO_PIN_7
#define E_STEP_GPIO_Port GPIOF
#define X_DIRECTION_Pin LL_GPIO_PIN_8
#define X_DIRECTION_GPIO_Port GPIOF
#define Y_DIRECTION_Pin LL_GPIO_PIN_9
#define Y_DIRECTION_GPIO_Port GPIOF
#define Z_DIRECTION_Pin LL_GPIO_PIN_10
#define Z_DIRECTION_GPIO_Port GPIOF
#define MCO_Pin LL_GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define TEST_PULSE_Pin LL_GPIO_PIN_0
#define TEST_PULSE_GPIO_Port GPIOC
#define RMII_MDC_Pin LL_GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define TEST_DIR_Pin LL_GPIO_PIN_2
#define TEST_DIR_GPIO_Port GPIOC
#define ENCODER5_A_Pin LL_GPIO_PIN_0
#define ENCODER5_A_GPIO_Port GPIOA
#define ENCODER5_B_Pin LL_GPIO_PIN_1
#define ENCODER5_B_GPIO_Port GPIOA
#define RMII_MDIO_Pin LL_GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define SPINDLE_PWM_Pin LL_GPIO_PIN_6
#define SPINDLE_PWM_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin LL_GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin LL_GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin LL_GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LD1_Pin LL_GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define COOLANT_FLOOD_Pin LL_GPIO_PIN_1
#define COOLANT_FLOOD_GPIO_Port GPIOB
#define COOLANT_MIST_Pin LL_GPIO_PIN_2
#define COOLANT_MIST_GPIO_Port GPIOB
#define A_DIRECTION_Pin LL_GPIO_PIN_11
#define A_DIRECTION_GPIO_Port GPIOF
#define B_DIRECTION_Pin LL_GPIO_PIN_12
#define B_DIRECTION_GPIO_Port GPIOF
#define C_DIRECTION_Pin LL_GPIO_PIN_13
#define C_DIRECTION_GPIO_Port GPIOF
#define D_DIRECTION_Pin LL_GPIO_PIN_14
#define D_DIRECTION_GPIO_Port GPIOF
#define E_DIRECTION_Pin LL_GPIO_PIN_15
#define E_DIRECTION_GPIO_Port GPIOF
#define ENCODER1_A_Pin LL_GPIO_PIN_9
#define ENCODER1_A_GPIO_Port GPIOE
#define ENCODER1_B_Pin LL_GPIO_PIN_11
#define ENCODER1_B_GPIO_Port GPIOE
#define STEPPERS_DISABLE_Pin LL_GPIO_PIN_12
#define STEPPERS_DISABLE_GPIO_Port GPIOB
#define RMII_TXD1_Pin LL_GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define LD3_Pin LL_GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin LL_GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin LL_GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define ENCODER4_A_Pin LL_GPIO_PIN_12
#define ENCODER4_A_GPIO_Port GPIOD
#define ENCODER4_B_Pin LL_GPIO_PIN_13
#define ENCODER4_B_GPIO_Port GPIOD
#define SPINDLE_ENABLE_Pin LL_GPIO_PIN_2
#define SPINDLE_ENABLE_GPIO_Port GPIOG
#define SPINDLE_DIRECTION_Pin LL_GPIO_PIN_3
#define SPINDLE_DIRECTION_GPIO_Port GPIOG
#define USB_PowerSwitchOn_Pin LL_GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin LL_GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define ENCODER8_A_Pin LL_GPIO_PIN_6
#define ENCODER8_A_GPIO_Port GPIOC
#define ENCODER8_B_Pin LL_GPIO_PIN_7
#define ENCODER8_B_GPIO_Port GPIOC
#define USB_SOF_Pin LL_GPIO_PIN_8
#define USB_SOF_GPIO_Port GPIOA
#define USB_VBUS_Pin LL_GPIO_PIN_9
#define USB_VBUS_GPIO_Port GPIOA
#define USB_ID_Pin LL_GPIO_PIN_10
#define USB_ID_GPIO_Port GPIOA
#define USB_DM_Pin LL_GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin LL_GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define ENCODER2_A_Pin LL_GPIO_PIN_15
#define ENCODER2_A_GPIO_Port GPIOA
#define SAFETY_DOOR_Pin LL_GPIO_PIN_10
#define SAFETY_DOOR_GPIO_Port GPIOC
#define SAFETY_DOOR_EXTI_IRQn EXTI15_10_IRQn
#define FEED_HOLD_Pin LL_GPIO_PIN_11
#define FEED_HOLD_GPIO_Port GPIOC
#define FEED_HOLD_EXTI_IRQn EXTI15_10_IRQn
#define CYCLE_START_Pin LL_GPIO_PIN_12
#define CYCLE_START_GPIO_Port GPIOC
#define CYCLE_START_EXTI_IRQn EXTI15_10_IRQn
#define X_LIMIT_Pin LL_GPIO_PIN_0
#define X_LIMIT_GPIO_Port GPIOD
#define X_LIMIT_EXTI_IRQn EXTI0_IRQn
#define Y_LIMIT_Pin LL_GPIO_PIN_1
#define Y_LIMIT_GPIO_Port GPIOD
#define Y_LIMIT_EXTI_IRQn EXTI1_IRQn
#define Z_LIMIT_Pin LL_GPIO_PIN_2
#define Z_LIMIT_GPIO_Port GPIOD
#define Z_LIMIT_EXTI_IRQn EXTI2_IRQn
#define A_LIMIT_Pin LL_GPIO_PIN_3
#define A_LIMIT_GPIO_Port GPIOD
#define A_LIMIT_EXTI_IRQn EXTI3_IRQn
#define B_LIMIT_Pin LL_GPIO_PIN_4
#define B_LIMIT_GPIO_Port GPIOD
#define B_LIMIT_EXTI_IRQn EXTI4_IRQn
#define C_LIMIT_Pin LL_GPIO_PIN_5
#define C_LIMIT_GPIO_Port GPIOD
#define C_LIMIT_EXTI_IRQn EXTI9_5_IRQn
#define D_LIMIT_Pin LL_GPIO_PIN_6
#define D_LIMIT_GPIO_Port GPIOD
#define D_LIMIT_EXTI_IRQn EXTI9_5_IRQn
#define E_LIMIT_Pin LL_GPIO_PIN_7
#define E_LIMIT_GPIO_Port GPIOD
#define E_LIMIT_EXTI_IRQn EXTI9_5_IRQn
#define PROBE_Pin LL_GPIO_PIN_9
#define PROBE_GPIO_Port GPIOG
#define RMII_TX_EN_Pin LL_GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin LL_GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define ENCODER2_B_Pin LL_GPIO_PIN_3
#define ENCODER2_B_GPIO_Port GPIOB
#define ENCODER3_A_Pin LL_GPIO_PIN_4
#define ENCODER3_A_GPIO_Port GPIOB
#define ENCODER3_B_Pin LL_GPIO_PIN_5
#define ENCODER3_B_GPIO_Port GPIOB
#define LD2_Pin LL_GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define SEGMENT_BUFFER_SIZE 6 // Uncomment to override default in stepper.h.

#ifdef USE_LINE_NUMBERS  // Uncomment to override default in planer.h.
  #define BLOCK_BUFFER_SIZE 15
#else
  #define BLOCK_BUFFER_SIZE 16
#endif
#ifndef USER_NON_VOLATILE
//  #define USER_NON_VOLATILE
#endif
#ifndef USER_RAPID_OVR_HIGH
  #define USER_RAPID_OVR_HIGH
#endif
#ifndef USER_DOUBLE
  #define USER_DOUBLE //in Symbol define
//  #define USER_DOUBLE_INCIDENTAL
#endif
#ifndef USER_MORE_AXIS
  //#define USER_MORE_AXIS //in Symbol define
#endif
#ifndef USER_6_AXIS
  #define USER_6_AXIS //in Symbol define
  #ifndef USER_8_AXIS
    #define USER_8_AXIS //in Symbol define
  #endif
#endif
#ifndef USER_BACKLASH
  //#define USER_BACKLASH //in Symbol define
  //#define USER_ACC_BACKLASH
#endif
#ifndef USER_STEP_PWM
  //#define USER_STEP_PWM //in Symbol define
#endif
#ifndef STEP_PULSE_DELAY
  #define STEP_PULSE_DELAY // Step pulse delay in microseconds. Default disabled.
#endif
//#define CHECK_PROBE_IN_INTERRUPT
//#define VARIABLE_SPINDLE // Default enabled. Comment to disable.
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
