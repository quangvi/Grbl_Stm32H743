/**
  ******************************************************************************
  * @file Encoder.h
  * @author SangTN - FPT Company
  * @version V1.0
  * @date 11-14-2018
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */
  /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_H
#define __ENCODER_H

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
void EncoderInit(void);
void ComputeEncoder(void);
void EncoderDetect(void);
void EncoderReset(void);
void IncreaseNumber(uint8_t Set);
void DecreaseNumber(uint8_t Set);
void EdgeTriggerDetection(uint8_t Set);
void ClearButton(void);
void ReadButton(void);
#ifdef __cplusplus
}
#endif
#endif /*__ENCODER_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

