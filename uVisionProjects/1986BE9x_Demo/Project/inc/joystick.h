/**
  ******************************************************************************
  * @file    joystick.h
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    09.09.2010
  * @brief   This file contains all the functions prototypes for the Joystick driver.
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
#ifndef __JOYSTICK_H
#define __JOYSTICK_H

/* Includes ------------------------------------------------------------------*/
#include <1986be9x.h>
#include "types.h"
#include "1986be9x_board.h"

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @defgroup Joystick_Driver Joystick Driver
  * @{
  */

/** @defgroup Joystick_Exported_Types Joystick Exported Types
  * @{
  */

/* "Key codes" */
typedef enum
{
  SEL        = 0,
  UP         = 1,
  DOWN       = 2,
  LEFT       = 3,
  RIGHT      = 4,
  NOKEY      = 5,
  MULTIPLE   = 6,
  NUM_KEY_CODES
}KeyCode;

/** @} */ /* End of group Joystick_Exported_Types */

/** @defgroup Joystick_Exported_Constants Joystick Exported Constants
  * @{
  */

/* Joystick control port pins definitions */

#if defined (USE_1986BE91_Rev0) || defined (USE_1986BE91_Rev1)
#define SEL_PORT           PORTC          /*!< SEL key port for 1986BE91 evaluation board */
#define SEL_PIN            PORT_Pin_10    /*!< SEL key pin for 1986BE91 evaluation board */
#define UP_PORT            PORTC          /*!< UP key port for 1986BE91 evaluation board */
#define UP_PIN             PORT_Pin_11    /*!< UP key pin for 1986BE91 evaluation board */
#define DOWN_PORT          PORTC          /*!< DOWN key port for 1986BE91 evaluation board */
#define DOWN_PIN           PORT_Pin_12    /*!< DOWN key pin for 1986BE91 evaluation board */
#define LEFT_PORT          PORTC          /*!< LEFT key port for 1986BE91 evaluation board */
#define LEFT_PIN           PORT_Pin_13    /*!< LEFT key pin for 1986BE91 evaluation board */
#define RIGHT_PORT         PORTC          /*!< RIGHT key port for 1986BE91 evaluation board */
#define RIGHT_PIN          PORT_Pin_14    /*!< RIGHT key pin for 1986BE91 evaluation board */

#elif defined (USE_1986BE92Y_Rev0)
#define SEL_PORT           PORTC
#define SEL_PIN            PORT_Pin_2
#define UP_PORT            PORTD
#define UP_PIN             PORT_Pin_5
#define DOWN_PORT          PORTE
#define DOWN_PIN           PORT_Pin_1
#define LEFT_PORT          PORTE
#define LEFT_PIN           PORT_Pin_3
#define RIGHT_PORT         PORTF
#define RIGHT_PIN          PORT_Pin_6

#elif defined (USE_1986BE92Y_Rev1)
#define SEL_PORT           PORTC
#define SEL_PIN            PORT_Pin_2
#define UP_PORT            PORTB
#define UP_PIN             PORT_Pin_5
#define DOWN_PORT          PORTE
#define DOWN_PIN           PORT_Pin_1
#define LEFT_PORT          PORTE
#define LEFT_PIN           PORT_Pin_3
#define RIGHT_PORT         PORTB
#define RIGHT_PIN          PORT_Pin_6

#elif defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)
#define SEL_PORT           PORTC
#define SEL_PIN            PORT_Pin_0
#define UP_PORT            PORTD
#define UP_PIN             PORT_Pin_0
#define DOWN_PORT          PORTD
#define DOWN_PIN           PORT_Pin_1
#define LEFT_PORT          PORTE
#define LEFT_PIN           PORT_Pin_3
#define RIGHT_PORT         PORTE
#define RIGHT_PIN          PORT_Pin_6

#endif

/** @} */ /* End of group Joystick_Exported_Constants */

/** @defgroup Joystick_Exported_Macros Joystick Exported Macros
  * @{
  */

#define KEY_PRESSED(x)              (GetKey() == x)
#define WAIT_UNTIL_KEY_PRESSED(x)   while(!KEY_PRESSED(x)){}
#define WAIT_UNTIL_KEY_RELEASED(x)  while(KEY_PRESSED(x)){}
#define WAIT_UNTIL_ANY_KEY          while(GetKey() == NOKEY){}

/** @} */ /* End of group Joystick_Exported_Macros */

/** @defgroup Joystick_Exported_Functions Joystick Exported Functions
  * @{
  */

KeyCode GetKey(void);

/** @} */ /* End of group Joystick_Exported_Functions */

/** @} */ /* End of group Joystick_Driver */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

#endif /* __JOYSTICK_H */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE joystick.h */

