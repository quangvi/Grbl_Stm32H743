/**
  ******************************************************************************
  * @file Delay.h
  * @author Nguyen_Sang - HK Company
  * @version V141119
  * @date 19-11-2014
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

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
void Delay_Init(void);
void delay_us(uint16_t us);
void delay_ms(uint16_t ms);
	 
#ifdef __cplusplus
}
#endif
#endif /*__DELAY_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

