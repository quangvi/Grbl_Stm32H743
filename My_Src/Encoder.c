/**
  ******************************************************************************
  * @file Encoder.c
  * @author SangTN - FPT Company
  * @version V1.0
  * @date 11-14-2018
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
  /* Includes ------------------------------------------------------------------*/
#include "Encoder.h"
#include "Extern_Variables_functions.h"

int32_t CountTimer1Enc = 0;
int32_t CountTimer1EncOver = 0;
int32_t CountTimer2Enc = 0;
int32_t CountTimer2EncOver = 0;
int32_t CountTimer3Enc = 0;
int32_t CountTimer3EncOver = 0;
int32_t CountTimer4Enc = 0;
int32_t CountTimer4EncOver = 0;
int32_t CountTimer5Enc = 0;
int32_t CountTimer5EncOver = 0;
int32_t CountTimer8Enc = 0;
int32_t CountTimer8EncOver = 0;

int32_t VelocityRotor3 = 0, PositionFeedback3 = 0,
    PositionDisplayPrevious3 = 0, temp_Velocity_Rpm3 = 0;
float Velocity_Rpm3 = 0;
int32_t VelocityRotor1 = 0, PositionFeedback1 = 0,
    PositionDisplayPrevious1 = 0, temp_Velocity_Rpm1 = 0;
float Velocity_Rpm1 = 0;

uint8_t Count = 0;
/*
int32_t NumberButton[4] = {0, 0, 0, 0};
int32_t StateButtonNew[4] = {0, 0, 0, 0};
int32_t StateButtonOld[4] = {0, 0, 0, 0};
uint8_t ExternalButtonFlag[4] = {0, 0, 0, 0};
int32_t NumberSet= 100;//200ms detection
*/

void EncoderInit(void)
{
  #ifdef USER_ENCODER_TEST
    TIM_Encoder_InitTypeDef sConfig;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_BOTHEDGE;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 5;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 5;
    #ifdef USER_STEP_PWM
    #else
      htim1.Instance = TIM1;
      htim1.Init.Prescaler = 1;
      htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim1.Init.Period = 0xFFFF;
      htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      #ifdef STM32H743xx
        htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
      #endif
      if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      //htim3.Instance->CNT = 0x8000;
      __HAL_TIM_SET_COUNTER(&htim1, 0x8000);
      HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
      HAL_TIM_Base_Start_IT(&htim1);
      //HAL_TIM_Base_Start(&htim1);
    #endif
/////////////////////////////////////////////////////////////////////////
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 1;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    #ifdef STM32H743xx
      htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    #endif

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&htim2, 0x80000000);
    //    htim2.Instance->CNT = 0x80000000;
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_IT(&htim2, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim2);
/////////////////////////////////////////////////////////////////////////
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    #ifdef STM32H743xx
      htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    #endif
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    //htim3.Instance->CNT = 0x8000;
    __HAL_TIM_SET_COUNTER(&htim3, 0x8000);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim3);
    //HAL_TIM_Base_Start(&htim3);
/////////////////////////////////////////////////////////////////////////
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 1;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xFFFF;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    #ifdef STM32H743xx
      htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    #endif
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    //htim3.Instance->CNT = 0x8000;
    __HAL_TIM_SET_COUNTER(&htim4, 0x8000);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    HAL_TIM_Base_Start_IT(&htim4);
    //HAL_TIM_Base_Start(&htim4);
/////////////////////////////////////////////////////////////////////////
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 1;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    #ifdef STM32H743xx
      htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    #endif

    if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
    {
      Error_Handler();
    }
    __HAL_TIM_SET_COUNTER(&htim5, 0x80000000);
    //    htim2.Instance->CNT = 0x80000000;
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_UPDATE);
    HAL_TIM_Base_Start(&htim5);
/////////////////////////////////////////////////////////////////////////
    #ifdef USER_STEP_PWM
    #else
      htim8.Instance = TIM8;
      htim8.Init.Prescaler = 1;
      htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
      htim8.Init.Period = 0xFFFF;
      htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
      #ifdef STM32H743xx
        htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
      #endif
      if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
      {
        Error_Handler();
      }
      //htim3.Instance->CNT = 0x8000;
      __HAL_TIM_SET_COUNTER(&htim8, 0x8000);
      HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);
      __HAL_TIM_CLEAR_IT(&htim8, TIM_IT_UPDATE);
      HAL_TIM_Base_Start_IT(&htim8);
      //HAL_TIM_Base_Start(&htim8);
    #endif
    /*
    This bit-field defines the frequency used to sample TI1 input and the length of the digital filter applied
    to TI1. The digital filter is made of an event counter in which N consecutive events are needed to
    validate a transition on the output:
    0000: No filter, sampling is done at fDTS
    0001: fSAMPLING=fCK_INT, N=2
    0010: fSAMPLING=fCK_INT, N=4
    0011: fSAMPLING=fCK_INT, N=8
    0100: fSAMPLING=fDTS/2, N=6
    0101: fSAMPLING=fDTS/2, N=8
    0110: fSAMPLING=fDTS/4, N=6
    0111: fSAMPLING=fDTS/4, N=8
    1000: fSAMPLING=fDTS/8, N=6
    1001: fSAMPLING=fDTS/8, N=8
    1010: fSAMPLING=fDTS/16, N=5
    1011: fSAMPLING=fDTS/16, N=6
    1100: fSAMPLING=fDTS/16, N=8
    1101: fSAMPLING=fDTS/32, N=5
    1110: fSAMPLING=fDTS/32, N=6
    1111: fSAMPLING=fDTS/32, N=8
    */
  #endif
}
void ComputeEncoder(void)
{
  Count++;
  if(Count >= 100)
  {
    Count = 0;
    CountTimer1Enc = (int16_t)(htim1.Instance->CNT - 0x8000) + CountTimer1EncOver;
    CountTimer2Enc = (int32_t)(htim2.Instance->CNT - 0x80000000);// + CountTimer2EncOver;
    CountTimer3Enc = (int16_t)(htim3.Instance->CNT - 0x8000) + CountTimer3EncOver;
    CountTimer4Enc = (int16_t)(htim4.Instance->CNT - 0x8000) + CountTimer4EncOver;
    CountTimer5Enc = (int32_t)(htim5.Instance->CNT - 0x80000000);// + CountTimer5EncOver;
    CountTimer8Enc = (int16_t)(htim8.Instance->CNT - 0x8000) + CountTimer8EncOver;

    PositionFeedback3 = CountTimer3Enc;
    VelocityRotor3 = (int32_t)(PositionFeedback3 - PositionDisplayPrevious3);
    PositionDisplayPrevious3 = PositionFeedback3;
  //    temp_Velocity_Rpm = VelocityRotor*60/(EncoderPulse * Multiply * SampleTime(ms/1000ms));
//    VelocityRotor3 = VelocityRotor3 *60; //--> min
//    temp_Velocity_Rpm3 = VelocityRotor3 * 10;//(100ms/1000ms)
    Velocity_Rpm3 = ((float)VelocityRotor3*600)/5000;//NumPulseEncoder;

    PositionFeedback1 = CountTimer1Enc;
    VelocityRotor1 = (int32_t)(PositionFeedback1 - PositionDisplayPrevious1);
    PositionDisplayPrevious1 = PositionFeedback1;
    Velocity_Rpm1 = ((float)VelocityRotor1*600)/5000;//NumPulseEncoder;
  }
}
/*
void IncreaseNumber(uint8_t Set)
{
  if(NumberButton[Set] < NumberSet)NumberButton[Set]++;
  else StateButtonNew[Set] = 1;
}

void DecreaseNumber(uint8_t Set)
{
  if(NumberButton[Set] > -NumberSet)NumberButton[Set]--;
  else StateButtonNew[Set] = 0;
}
void EdgeTriggerDetection(uint8_t Set)
{
  #if defined (POLARITY_LOW)
    if((StateButtonOld[Set] == 1) && (StateButtonNew[Set] == 0))//Falling edge
  #elif defined (POLARITY_HIGH)
    if((StateButtonOld[Set] == 0) && (StateButtonNew[Set] == 1))//Rising edge
  #else
    #error "Please select first the polarity trigger used in your application (in Extern_Variables_functions.h file)"
  #endif
    {
        ExternalButtonFlag[Set] = 1;
    }
//    if(StateButtonOld != StateButtonNew)
//    {
      StateButtonOld[Set] = StateButtonNew[Set]; //save State
//    }
}

void ClearButton(void)
{
  uint8_t aa = COUNTOF(ExternalButtonFlag);//sizeof(uint8_t)*4
  memset(ExternalButtonFlag, 0, aa);// Clear
}

void ReadButton(void)
{
  if(__HAL_GPIO_READ_BITS(ENC_SW_GPIO_Port, ENC_SW_Pin) != GPIO_PIN_RESET){
    IncreaseNumber(ENC_SW);}
  else  DecreaseNumber(ENC_SW);

  EdgeTriggerDetection(ENC_SW);

  if(__HAL_GPIO_READ_BITS(KEY_0_GPIO_Port, KEY_0_Pin) != GPIO_PIN_RESET){
    IncreaseNumber(KEY_0);}
  else DecreaseNumber(KEY_0);

  EdgeTriggerDetection(KEY_0);

  if(__HAL_GPIO_READ_BITS(KEY_1_GPIO_Port, KEY_1_Pin) != GPIO_PIN_RESET){
    IncreaseNumber(KEY_1);}
  else DecreaseNumber(KEY_1);

  EdgeTriggerDetection(KEY_1);

  if(__HAL_GPIO_READ_BITS(KEY_UP_GPIO_Port, KEY_UP_Pin) != GPIO_PIN_RESET){
    IncreaseNumber(KEY_UP);}
  else DecreaseNumber(KEY_UP);

  EdgeTriggerDetection(KEY_UP);
}
*/
