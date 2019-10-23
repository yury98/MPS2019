/**
  ******************************************************************************
  * @file    1986BE9x_it.h
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    08.09.2010
  * @brief   This file contains all the prototypes of interrupt handler functions
  ******************************************************************************
  * <br><br>
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 Phyton</center></h2>
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __1986BE9x_H
#define __1986BE9x_H

/* Includes ------------------------------------------------------------------*/
#include <1986be9x.h>
#include "types.h"

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @defgroup Interrupt_Service_Routines Interrupt Service Routines
  * @{
  */

/** @defgroup Interrupt_Service_Exported_Types Interrupt Service Exported Types
  * @{
  */

typedef void (* tUARTFunc)(void);
typedef void (* tUARTLineStateFunc)(uint32_t);

/** @} */ /* End of group Interrupt_Service_Exported_Types */

/** @defgroup Interrupt_Service_Exported_Macros Interrupt Service Exported Macros
  * @{
  */

/* UART event mask as of interest by CDC PSTN line state notifications */
#define UART_LINE_STATE_EVENTS		(UART_IT_OE | UART_IT_BE | UART_IT_PE | UART_IT_FE | \
																	 UART_IT_DSR | UART_IT_DCD | UART_IT_RI)

/** @} */ /* End of group Interrupt_Service_Exported_Macros */


/** @defgroup Interrupt_Service_Exported_Variables Interrupt Service Exported Variables
  * @{
  */

/* Timer counter */
extern vuint32_t TimerCounter;

/* Current value of the ADC1_RESULT register */
extern vuint32_t ADC1_Value;

/* Pointers to UART send/receive interrupt handlers */
extern tUARTFunc pfUARTSenderFunc;
extern tUARTFunc pfUARTReceiverFunc;
extern tUARTLineStateFunc pfUARTLineStateFunc;

/* Stop mode flag */
extern vuint32_t STOPModeStatus;

/* Alarm flag */
extern vuint32_t AlarmSetStatus;

/** @} */ /* End of group Interrupt_Service_Exported_Variables */

/** @} */ /* End of group Interrupt_Service_Routines */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

#endif /* __1986BE9x_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE 1986BE9x_it.h */

