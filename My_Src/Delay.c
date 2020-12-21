/**
  ******************************************************************************
  * @file Delay.c
  * @author Nguyen_Sang - HK Company
  * @version V141119
  * @date 19-11-2014
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
	/* Includes ------------------------------------------------------------------*/
#include "Delay.h"
#include "Extern_Variables_functions.h"

void Delay_Init(void)
{
  /*##- Start the TIM11 Base generation for Delay_2  ####################*/
  RCC_ClkInitTypeDef    clkconfig;
  uint32_t              uwTimclock = 0;
  uint32_t              uwPrescalerValue = 0;
  uint32_t              pFLatency;
  /* Get clock configuration */
  HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

  /* Compute TIM clock */
  uwTimclock = 2*HAL_RCC_GetPCLK1Freq();

  /* Compute the prescaler value to have TIM counter clock equal to 1MHz */
  //(((SystemCoreClock / APB1_PRESCALE) * 2) / 1000000) - 1;//83;
  uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000) - 1);

//  /* Set the Auto-reload value */
//  htim6.Instance->ARR = 0xFFFF;
//  /* Set the Prescaler value */
//  htim6.Instance->PSC = uwPrescalerValue;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = uwPrescalerValue;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xFFFF;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(&htim6);

  /* Compute the prescaler value to have TIM counter clock equal to 100MHz */
  //(((SystemCoreClock / APB1_PRESCALE) * 2) / 100000000) - 1;//83;
  uwPrescalerValue = (uint32_t) ((uwTimclock / 100000000) - 1);

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = uwPrescalerValue;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 0xFFFF;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    /* Starting Error */
    Error_Handler();
  }
  /* Enable the Peripheral */
  __HAL_TIM_ENABLE(&htim7);
}
void delay_us(uint16_t us)
{
  htim6.Instance->CNT=0;
  while(us>htim6.Instance->CNT);
}

void delay_ms(uint16_t ms)
{ 
  while(ms--)delay_us(1000);
}
