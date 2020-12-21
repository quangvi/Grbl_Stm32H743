/**
  ******************************************************************************
  * @file Config_stm32f4.h
  * @author Nguyen_Sang - HongKy Company
  * @version V1.0
  * @date 09-05-2015
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONFIG_STM32F4_H
#define __CONFIG_STM32F4_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
  #include "main.h"
  #include "Extern_Variables_functions.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Private function prototypes -----------------------------------------------*/
  void MyLIMIT_HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
  #ifdef STM32H743xx
    void MyHAL_UART_IRQHandlerRxTx(void);
  #endif
  void MyHAL_UART_IRQHandlerTx(void);
  void MyHAL_UART_IRQHandlerRx(void);
#ifdef __cplusplus
}
#endif
#endif /*__CONFIG_H */


/**
  * @}
  */ 

/**
  * @}
  */ 

/*****************************END OF FILE****/
