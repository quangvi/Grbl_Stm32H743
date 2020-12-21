/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h7xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Extern_Variables_functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
u32 LevelCountStep = 0;
u16 CountTimer7Max = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim13;
extern UART_HandleTypeDef huart3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */
  #ifdef USER_ENCODER_TEST
    ComputeEncoder();
  #endif
  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles RCC global interrupt.
  */
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI0_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);
    /* USER CODE BEGIN LL_EXTI_LINE_0 */
    
    /* USER CODE END LL_EXTI_LINE_0 */
  }
  /* USER CODE BEGIN EXTI0_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET)
  #endif
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
//    ISR_LIMIT_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI1_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);
    /* USER CODE BEGIN LL_EXTI_LINE_1 */
    
    /* USER CODE END LL_EXTI_LINE_1 */
  }
  /* USER CODE BEGIN EXTI1_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
  #endif
  if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_1) != RESET)
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_1);
//    ISR_LIMIT_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI2_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_2) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_2);
    /* USER CODE BEGIN LL_EXTI_LINE_2 */
    
    /* USER CODE END LL_EXTI_LINE_2 */
  }
  /* USER CODE BEGIN EXTI2_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_2) != RESET)
  #endif
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
//    ISR_LIMIT_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */
    
    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI3_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_3) != RESET)
  #endif
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
//    ISR_LIMIT_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI4_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_4) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_4);
    /* USER CODE BEGIN LL_EXTI_LINE_4 */
    
    /* USER CODE END LL_EXTI_LINE_4 */
  }
  /* USER CODE BEGIN EXTI4_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_4) != RESET)
  #endif
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
//    ISR_LIMIT_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI9_5_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_5) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_5);
    /* USER CODE BEGIN LL_EXTI_LINE_5 */
    
    /* USER CODE END LL_EXTI_LINE_5 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_6) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_6);
    /* USER CODE BEGIN LL_EXTI_LINE_6 */
    
    /* USER CODE END LL_EXTI_LINE_6 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_7) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_7);
    /* USER CODE BEGIN LL_EXTI_LINE_7 */
    
    /* USER CODE END LL_EXTI_LINE_7 */
  }
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7) != RESET)
  #endif
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_7);
//    ISR_LIMIT_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
#else //not USER_HALL_IT
    if(__HAL_TIM_GET_FLAG(&htim1, TIM_FLAG_UPDATE) != RESET)
    {
      #ifdef USER_ACTIVE_IT
        if(__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_UPDATE) !=RESET)
      #endif //USER_ACTIVE_FLAG
      {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1)) {
          CountTimer1EncOver -= 65536;
        }
        else {
          CountTimer1EncOver += 65536;
        }
        __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
      }
    }
#endif //USER_HALL_IT
  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
#else //not USER_HALL_IT
    if(__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_UPDATE) != RESET)
    {
      #ifdef USER_ACTIVE_IT
        if(__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_UPDATE) !=RESET)
      #endif //USER_ACTIVE_FLAG
      {
        if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)) {
          CountTimer3EncOver -= 65536;
        }
        else {
          CountTimer3EncOver += 65536;
        }
        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
      }
    }
#endif //USER_HALL_IT
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
#else //not USER_HALL_IT
  if(__HAL_TIM_GET_FLAG(&htim4, TIM_FLAG_UPDATE) != RESET)
  {
    #ifdef USER_ACTIVE_IT
      if(__HAL_TIM_GET_IT_SOURCE(&htim4, TIM_IT_UPDATE) !=RESET)
    #endif //USER_ACTIVE_FLAG
    {
      if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
        CountTimer4EncOver -= 65536;
      }
      else {
        CountTimer4EncOver += 65536;
      }
      __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    }
  }
#endif //USER_HALL_IT
  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef STM32H743xx
    MyHAL_UART_IRQHandlerRxTx();
    LD2_GPIO_Port->ODR ^= LD2_Pin;
  #else
    MyHAL_UART_IRQHandlerRx();
    LD3_GPIO_Port->ODR ^= LD3_Pin;
  #endif
#endif //USER_HALL_IT
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END EXTI15_10_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
    /* USER CODE BEGIN LL_EXTI_LINE_10 */
    
    /* USER CODE END LL_EXTI_LINE_10 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_11) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_11);
    /* USER CODE BEGIN LL_EXTI_LINE_11 */
    
    /* USER CODE END LL_EXTI_LINE_11 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
    /* USER CODE BEGIN LL_EXTI_LINE_12 */
    
    /* USER CODE END LL_EXTI_LINE_12 */
  }
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_13) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
    /* USER CODE BEGIN LL_EXTI_LINE_13 */
    
    /* USER CODE END LL_EXTI_LINE_13 */
  }
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */
#else //not USER_HALL_IT
  #ifdef USER_ACTIVE_IT
    if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13) != RESET)
  #endif
  {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_11);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13);
//    ISR_CONTROL_INT_vect();
  }
#endif //USER_HALL_IT
  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
  #ifdef COUNT_TIMER
    __HAL_TIM_SET_COUNTER(&htim7, 0);//
  #endif
  #ifdef TEST_PULSE
    __HAL_GPIO_SET_BITS(TEST_PULSE_GPIO_Port, TEST_PULSE_Pin);
  #endif
  #ifdef USER_DISPLAY_PULSE_LED
    #ifdef STM32H743xx
      //__HAL_GPIO_SET_BITS(LD3_GPIO_Port, LD3_Pin);
      LL_GPIO_SetOutputPin(LD3_GPIO_Port, LD3_Pin);
    #else
      //__HAL_GPIO_SET_BITS(LD4_GPIO_Port, LD4_Pin);
      LL_GPIO_SetOutputPin(LD4_GPIO_Port, LD4_Pin);
    #endif
  #endif
  #ifdef USER_HALL_IT
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */
  #else //not USER_HALL_IT
    #ifdef USER_ACTIVE_FLAG
      if(__HAL_TIM_GET_FLAG(&htim12, TIM_FLAG_UPDATE) != RESET)
    #endif //USER_ACTIVE_FLAG
      {
        #ifdef USER_ACTIVE_IT
          if(__HAL_TIM_GET_IT_SOURCE(&htim12, TIM_IT_UPDATE) !=RESET)
        #endif //USER_ACTIVE_FLAG
        {
          //__HAL_TIM_CLEAR_IT(&htim12, TIM_IT_UPDATE);
          LL_TIM_ClearFlag_UPDATE(TIM12);

          ISR_TIMER1_COMPA_vect();
        }
      }
    #endif //USER_HALL_IT
    #ifdef TEST_PULSE
      __HAL_GPIO_RESET_BITS(TEST_PULSE_GPIO_Port, TEST_PULSE_Pin);
    #endif
    #ifdef USER_DISPLAY_PULSE_LED
      #ifdef STM32H743xx
        //__HAL_GPIO_RESET_BITS(LD3_GPIO_Port, LD3_Pin);
        LL_GPIO_ResetOutputPin(LD3_GPIO_Port, LD3_Pin);
      #else
        //__HAL_GPIO_RESET_BITS(LD4_GPIO_Port, LD4_Pin);
        LL_GPIO_ResetOutputPin(LD4_GPIO_Port, LD4_Pin);
      #endif
    #endif
    #ifdef COUNT_TIMER
//    /* TIM Break input event */
//    if(__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_BREAK) != RESET)
//    {
//      __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_BREAK);
//    }
//    /* TIM Break input2 event */
//    if(__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_BREAK2) != RESET)
//    {
//      __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_BREAK);
//    }
    uint16_t CountTimer7 = __HAL_TIM_GET_COUNTER(&htim7);
    //uint16_t CountTimer7 = LL_TIM_GetCounter(TIM7);
    if(CountTimer7Max < CountTimer7)//15827
    {
      CountTimer7Max = CountTimer7;
    }
  #endif
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */
#ifdef USER_HALL_IT
  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */
#else //not USER_HALL_IT
  if(__HAL_TIM_GET_FLAG(&htim8, TIM_FLAG_UPDATE) != RESET)
  {
    #ifdef USER_ACTIVE_IT
      if(__HAL_TIM_GET_IT_SOURCE(&htim8, TIM_IT_UPDATE) !=RESET)
    #endif //USER_ACTIVE_FLAG
    {
      if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8)) {
        CountTimer8EncOver -= 65536;
      }
      else {
        CountTimer8EncOver += 65536;
      }
      __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
    }
//    if(__HAL_TIM_GET_FLAG(&htim13, TIM_FLAG_UPDATE) != RESET)
//    {
//      __HAL_TIM_CLEAR_IT(&htim13, TIM_IT_UPDATE);
//    }
  }
#endif //USER_HALL_IT
  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM8 trigger and commutation interrupts and TIM14 global interrupt.
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */

  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 1 */
  if(LL_TIM_IsActiveFlag_UPDATE(TIM14) == 1)
  {
    /* Clear the update interrupt flag*/
    LL_TIM_ClearFlag_UPDATE(TIM14);

    /* TIM14 update interrupt processing */
    //__HAL_TIM_CLEAR_IT(&htim14, TIM_IT_UPDATE);
    LL_TIM_ClearFlag_UPDATE(TIM14);

    #ifdef STEP_PULSE_DELAY
      #ifdef USER_STEP_PWM
      #else
        if(LevelCountStep == 0)
        {
          ISR_TIMER0_COMPA_vect();
          LevelCountStep = 1;
        }
        else
        {
          ISR_TIMER0_OVF_vect();
          LevelCountStep = 0;
        }
      #endif //USER_STEP_PWM
    #else //not STEP_PULSE_DELAY
      ISR_TIMER0_OVF_vect();
    #endif //STEP_PULSE_DELAY
  }
  /* USER CODE END TIM8_TRG_COM_TIM14_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
