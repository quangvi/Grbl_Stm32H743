/**
  ******************************************************************************
  * @file Config_stm32f4.c
  * @author Nguyen_Sang - HongKy Company
  * @version V1.0
  * @date 09-05-2015
  ******************************************************************************
  * @source
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "Config_stm32f4.h"
#include "Extern_Variables_functions.h"

/* Private variables ---------------------------------------------------------*/

#ifdef STM32H743xx
/**
  * @brief Handle UART interrupt request.
  * @param huart: UART handle.
  * @retval None
  */
void MyHAL_UART_IRQHandlerRxTx(void)
{
  uint32_t isrflags   = READ_REG(huart3.Instance->ISR);
  uint32_t cr1its     = READ_REG(huart3.Instance->CR1);
  uint32_t cr3its     = READ_REG(huart3.Instance->CR3);
  uint32_t errorflags;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
     && (   ((cr1its & USART_CR1_RXNEIE_RXFNEIE) != RESET)
         || ((cr3its & USART_CR3_RXFTIE) != RESET)) )
    {
//      UART_Receive_IT(huart);
      ISR_SERIAL_RX();
      return;
    }
  }

  /* If some errors occur */
  if(   (errorflags != RESET)
     && (   ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != RESET)
         || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_PEF);

      huart3.ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_FEF);

      huart3.ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_NEF);

      huart3.ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(   ((isrflags & USART_ISR_ORE) != RESET)
        &&(  ((cr1its & USART_CR1_RXNEIE) != RESET) ||
             ((cr3its & USART_CR3_RXFTIE) != RESET) ||
             ((cr3its & USART_CR3_EIE) != RESET)) )
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_OREF);

      huart3.ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be --------------------------*/
    if(huart3.ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver ---------------------------------------------------*/
      if(((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
         && (   ((cr1its & USART_CR1_RXNEIE) != RESET)
             || ((cr3its & USART_CR3_RXFTIE) != RESET)) )
      {
        //UART_Receive_IT(huart);
    	  ISR_SERIAL_RX();
      }

//      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
//         consider error as blocking */
//      if (((huart3.ErrorCode & HAL_UART_ERROR_ORE) != RESET) ||
//          (HAL_IS_BIT_SET(huart3.Instance->CR3, USART_CR3_DMAR)))
//      {
//        /* Blocking error : transfer is aborted
//           Set the UART state ready to be able to start again the process,
//           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
//        UART_EndRxTransfer(huart);
//      }
//        /* Disable the UART DMA Rx request if enabled */
//        if (HAL_IS_BIT_SET(huart3.Instance->CR3, USART_CR3_DMAR))
//        {
//          CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
//
//          /* Abort the UART DMA Rx channel */
//          if(huart3.hdmarx != NULL)
//          {
//            /* Set the UART DMA Abort callback :
//               will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
//            huart3.hdmarx->XferAbortCallback = UART_DMAAbortOnError;
//
//            /* Abort DMA RX */
//            if(HAL_DMA_Abort_IT(huart3.hdmarx) != HAL_OK)
//            {
//              /* Call Directly huart3.hdmarx->XferAbortCallback function in case of error */
//              huart3.hdmarx->XferAbortCallback(huart3.hdmarx);
//            }
//          }
//          else
//          {
//            /* Call user error callback */
//            HAL_UART_ErrorCallback(huart);
//          }
//        }
//        else
//        {
//          /* Call user error callback */
//          HAL_UART_ErrorCallback(huart);
//        }
//      }
//      else
//      {
//        /* Non Blocking error : transfer could go on.
//           Error is notified to user through user error callback */
//        HAL_UART_ErrorCallback(huart);
        huart3.ErrorCode = HAL_UART_ERROR_NONE;
    }
    return;

  } /* End if some error occurs */

  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
/*   if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_WUF);
    // Set the UART state ready to be able to start again the process 
    huart3.gState  = HAL_UART_STATE_READY;
    huart3.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_WakeupCallback(&huart3);
    return;
  } */

  /* UART in mode Transmitter ------------------------------------------------*/
  if(((isrflags & USART_ISR_TXE_TXFNF) != RESET)
     && (   ((cr1its & USART_CR1_TXEIE) != RESET)
         || ((cr3its & USART_CR3_TXFTIE) != RESET)) )
  {
//    UART_Transmit_IT(huart);
    ISR_SERIAL_UDRE_TX();
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
//    UART_EndTransmit_IT(huart);
    /* Disable the UART Transmit Complete Interrupt */
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TCIE);
    return;
  }

  /* UART TX FIFO Empty  -----------------------------------------------------*/
  if(((isrflags & USART_ISR_TXFE) != RESET) && ((cr1its & USART_CR1_TXFEIE) != RESET))
  {
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TXFEIE);
  }
}
/**
  * @brief Handle UART interrupt request.
  * @param huart: UART handle.
  * @retval None
  */
void MyHAL_UART_IRQHandlerRx(void)
{
  uint32_t isrflags   = READ_REG(huart3.Instance->ISR);
  uint32_t cr1its     = READ_REG(huart3.Instance->CR1);
  uint32_t cr3its     = READ_REG(huart3.Instance->CR3);
  uint32_t errorflags;

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_ISR_PE | USART_ISR_FE | USART_ISR_ORE | USART_ISR_NE));
  if (errorflags == RESET)
  {
    /* UART in mode Receiver ---------------------------------------------------*/
    if(((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
     && (   ((cr1its & USART_CR1_RXNEIE_RXFNEIE) != RESET)
         || ((cr3its & USART_CR3_RXFTIE) != RESET)) )
    {
//      UART_Receive_IT(huart);
      ISR_SERIAL_RX();
      return;
    }
  }

  /* If some errors occur */
  if(   (errorflags != RESET)
     && (   ((cr3its & (USART_CR3_RXFTIE | USART_CR3_EIE)) != RESET)
         || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
  {
    /* UART parity error interrupt occurred -------------------------------------*/
    if(((isrflags & USART_ISR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_PEF);

      huart3.ErrorCode |= HAL_UART_ERROR_PE;
    }

    /* UART frame error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_FEF);

      huart3.ErrorCode |= HAL_UART_ERROR_FE;
    }

    /* UART noise error interrupt occurred --------------------------------------*/
    if(((isrflags & USART_ISR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_NEF);

      huart3.ErrorCode |= HAL_UART_ERROR_NE;
    }

    /* UART Over-Run interrupt occurred -----------------------------------------*/
    if(   ((isrflags & USART_ISR_ORE) != RESET)
        &&(  ((cr1its & USART_CR1_RXNEIE) != RESET) ||
             ((cr3its & USART_CR3_RXFTIE) != RESET) ||
             ((cr3its & USART_CR3_EIE) != RESET)) )
    {
      __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_OREF);

      huart3.ErrorCode |= HAL_UART_ERROR_ORE;
    }

    /* Call UART Error Call back function if need be --------------------------*/
    if(huart3.ErrorCode != HAL_UART_ERROR_NONE)
    {
      /* UART in mode Receiver ---------------------------------------------------*/
      if(((isrflags & USART_ISR_RXNE_RXFNE) != RESET)
         && (   ((cr1its & USART_CR1_RXNEIE_RXFNEIE) != RESET)
             || ((cr3its & USART_CR3_RXFTIE) != RESET)) )
      {
        //UART_Receive_IT(huart);
    	  ISR_SERIAL_RX();
      }

//      /* If Overrun error occurs, or if any error occurs in DMA mode reception,
//         consider error as blocking */
//      if (((huart6.ErrorCode & HAL_UART_ERROR_ORE) != RESET) ||
//          (HAL_IS_BIT_SET(huart6.Instance->CR3, USART_CR3_DMAR)))
//      {
//        /* Blocking error : transfer is aborted
//           Set the UART state ready to be able to start again the process,
//           Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
//        UART_EndRxTransfer(huart);
//      }
//        /* Disable the UART DMA Rx request if enabled */
//        if (HAL_IS_BIT_SET(huart6.Instance->CR3, USART_CR3_DMAR))
//        {
//          CLEAR_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
//
//          /* Abort the UART DMA Rx channel */
//          if(huart6.hdmarx != NULL)
//          {
//            /* Set the UART DMA Abort callback :
//               will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
//            huart6.hdmarx->XferAbortCallback = UART_DMAAbortOnError;
//
//            /* Abort DMA RX */
//            if(HAL_DMA_Abort_IT(huart6.hdmarx) != HAL_OK)
//            {
//              /* Call Directly huart6.hdmarx->XferAbortCallback function in case of error */
//              huart6.hdmarx->XferAbortCallback(huart6.hdmarx);
//            }
//          }
//          else
//          {
//            /* Call user error callback */
//            HAL_UART_ErrorCallback(huart);
//          }
//        }
//        else
//        {
//          /* Call user error callback */
//          HAL_UART_ErrorCallback(huart);
//        }
//      }
//      else
//      {
//        /* Non Blocking error : transfer could go on.
//           Error is notified to user through user error callback */
//        HAL_UART_ErrorCallback(huart);
        huart3.ErrorCode = HAL_UART_ERROR_NONE;
    }
    return;

  } /* End if some error occurs */

  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_WUF);
    /* Set the UART state ready to be able to start again the process */
    huart3.gState  = HAL_UART_STATE_READY;
    huart3.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_WakeupCallback(&huart3);
    return;
  }
}
#else

/**
  * @brief  This function handles UART interrupt request.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void MyHAL_UART_IRQHandlerRx(void)
{
  uint32_t isrflags   = READ_REG(huart3.Instance->SR);
  uint32_t cr1its     = READ_REG(huart3.Instance->CR1);
  uint32_t cr3its     = READ_REG(huart3.Instance->CR3);
  uint32_t errorflags = 0x00U;
//  uint32_t dmarequest = 0x00U;

 /* If no error occurs */
 errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
 if(errorflags == RESET)
 {
   /* UART in mode Receiver -------------------------------------------------*/
   if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
   {
     ISR_SERIAL_RX();
     //UART_Receive_IT(huart3);
//     /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
//     SET_BIT(huart3.Instance->CR3, USART_CR3_EIE);
//
//     /* Enable the UART Parity Error and Data Register not empty Interrupts */
//     SET_BIT(huart3.Instance->CR1, USART_CR1_PEIE | USART_CR1_RXNEIE);
//
//     /* Rx process is completed, restore huart->RxState to Ready */
//     huart3.RxState = HAL_UART_STATE_READY;
     return;
   }
 }

 /* If some errors occur */
 if((errorflags != RESET) && (((cr3its & USART_CR3_EIE) != RESET) || ((cr1its & (USART_CR1_RXNEIE | USART_CR1_PEIE)) != RESET)))
 {
//   /* UART parity error interrupt occurred ----------------------------------*/
//   if(((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
//   {
//     huart3.ErrorCode |= HAL_UART_ERROR_PE;
//   }
//
//   /* UART noise error interrupt occurred -----------------------------------*/
//   if(((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
//   {
//     huart3.ErrorCode |= HAL_UART_ERROR_NE;
//   }
//
//   /* UART frame error interrupt occurred -----------------------------------*/
//   if(((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
//   {
//     huart3.ErrorCode |= HAL_UART_ERROR_FE;
//   }
//
//   /* UART Over-Run interrupt occurred --------------------------------------*/
//   if(((isrflags & USART_SR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
//   {
//     huart3.ErrorCode |= HAL_UART_ERROR_ORE;
//   }

   /* Call UART Error Call back function if need be --------------------------*/
   if(huart3.ErrorCode != HAL_UART_ERROR_NONE)
   {
     /* UART in mode Receiver -----------------------------------------------*/
     if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
     {
       ISR_SERIAL_RX();
       //UART_Receive_IT(huart3);
     }

//     /* If Overrun error occurs, or if any error occurs in DMA mode reception,
//        consider error as blocking */
//     dmarequest = HAL_IS_BIT_SET(huart3.Instance->CR3, USART_CR3_DMAR);
//     if(((huart3.ErrorCode & HAL_UART_ERROR_ORE) != RESET) || dmarequest)
//     {
//       /* Blocking error : transfer is aborted
//          Set the UART state ready to be able to start again the process,
//          Disable Rx Interrupts, and disable Rx DMA request, if ongoing */
////       UART_EndRxTransfer(huart3);
//
////       /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
////       CLEAR_BIT(huart3.Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
////       CLEAR_BIT(huart3.Instance->CR3, USART_CR3_EIE);
//
//       /* At end of Rx process, restore huart->RxState to Ready */
//       huart3.RxState = HAL_UART_STATE_READY;
//       /* Disable the UART DMA Rx request if enabled */
//       if(HAL_IS_BIT_SET(huart3.Instance->CR3, USART_CR3_DMAR))
//       {
//         CLEAR_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
//
//         /* Abort the UART DMA Rx channel */
//         if(huart3.hdmarx != NULL)
//         {
//           /* Set the UART DMA Abort callback :
//              will lead to call HAL_UART_ErrorCallback() at end of DMA abort procedure */
////           huart3.hdmarx->XferAbortCallback = UART_DMAAbortOnError;
//           if(HAL_DMA_Abort_IT(huart3.hdmarx) != HAL_OK)
//           {
//             /* Call Directly XferAbortCallback function in case of error */
//             huart3.hdmarx->XferAbortCallback(huart3.hdmarx);
//           }
//         }
//         else
//         {
//           /* Call user error callback */
////           HAL_UART_ErrorCallback(huart3);
//         }
//       }
//       else
//       {
//         /* Call user error callback */
////         HAL_UART_ErrorCallback(huart3);
//       }
//     }
//     else
//     {
//       /* Non Blocking error : transfer could go on.
//          Error is notified to user through user error callback */
////       HAL_UART_ErrorCallback(huart3);
       huart3.ErrorCode = HAL_UART_ERROR_NONE;
//     }
   }
   return;
 } /* End if some error occurs */

// /* UART in mode Transmitter ------------------------------------------------*/
// if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
// {
////   UART_Transmit_IT(huart3);
   /* Disable the UART Transmit Complete Interrupt */
   CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TXEIE);

//   /* Enable the UART Transmit Complete Interrupt */
//   SET_BIT(huart3.Instance->CR1, USART_CR1_TCIE);
//   return;
// }

// /* UART in mode Transmitter end --------------------------------------------*/
// if(((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
// {
////   UART_EndTransmit_IT(huart3);
   /* Disable the UART Transmit Complete Interrupt */
   CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TCIE);

//   /* Tx process is ended, restore huart->gState to Ready */
//   huart3.gState = HAL_UART_STATE_READY;
//   return;
// }
}
#endif

#ifdef STM32H743xx
/**
  * @brief Handle UART interrupt request.
  * @param huart: UART handle.
  * @retval None
  */
void MyHAL_UART_IRQHandlerTx(void)
{
  uint32_t isrflags   = READ_REG(huart3.Instance->ISR);
  uint32_t cr1its     = READ_REG(huart3.Instance->CR1);
  uint32_t cr3its     = READ_REG(huart3.Instance->CR3);
  /* UART wakeup from Stop mode interrupt occurred ---------------------------*/
  if(((isrflags & USART_ISR_WUF) != RESET) && ((cr3its & USART_CR3_WUFIE) != RESET))
  {
    __HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_WUF);
    /* Set the UART state ready to be able to start again the process */
    huart3.gState  = HAL_UART_STATE_READY;
    huart3.RxState = HAL_UART_STATE_READY;
    HAL_UARTEx_WakeupCallback(&huart3);
    return;
  }

  /* UART in mode Transmitter ------------------------------------------------*/
  if(((isrflags & USART_ISR_TXE_TXFNF) != RESET)
     && (   ((cr1its & USART_CR1_TXEIE_TXFNFIE) != RESET)
         || ((cr3its & USART_CR3_TXFTIE) != RESET)) )
  {
    //UART_Transmit_IT(huart3);
    ISR_SERIAL_UDRE_TX();
    return;
  }

  /* UART in mode Transmitter (transmission end) -----------------------------*/
  if(((isrflags & USART_ISR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
//    UART_EndTransmit_IT(huart3);
    /* Disable the UART Transmit Complete Interrupt */
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TCIE);
    return;
  }

  /* UART TX FIFO Empty  -----------------------------------------------------*/
  if(((isrflags & USART_ISR_TXFE) != RESET) && ((cr1its & USART_CR1_TXFEIE) != RESET))
  {
    CLEAR_BIT(huart3.Instance->CR1, USART_CR1_TXFEIE);
  }
}
#else
/**
  * @brief  This function handles UART interrupt request.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void MyHAL_UART_IRQHandlerTx(void)
{
   uint32_t isrflags   = READ_REG(huart6.Instance->SR);
   uint32_t cr1its     = READ_REG(huart6.Instance->CR1);

  /* UART in mode Transmitter ------------------------------------------------*/
  if(((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
  {
    ISR_SERIAL_UDRE_TX();//UART_Transmit_IT(huart);
    return;
  }

  /* UART in mode Transmitter end --------------------------------------------*/
  if(((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
  {
    /* Disable the UART Transmit Complete Interrupt */
    CLEAR_BIT(huart6.Instance->CR1, USART_CR1_TCIE);//UART_EndTransmit_IT(huart);
    return;
  }
}
#endif
/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
