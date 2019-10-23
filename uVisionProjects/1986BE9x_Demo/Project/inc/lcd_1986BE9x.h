/**
  ******************************************************************************
  * @file    lcd_1986BE9x.h
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    10.09.2010
  * @brief   This file contains all the specific types, constants and variables
  *          for the LCD driver for 1986BE92Y_Rev0, 1986BE92Y_Rev1,
  *          1986BE93Y_Rev0 and 1986BE93Y_Rev1 evaluation boards.
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

#if defined (USE_1986BE92Y_Rev0)  || defined (USE_1986BE92Y_Rev1) || \
    defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_H
#define __LCD_H

/* Includes ------------------------------------------------------------------*/
#include <1986be9x.h>
#include <1986be9x_port.h>
#include <1986be9x_rst_clk.h>
#include "types.h"
#include "1986be9x_board.h"

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @addtogroup LCD_Driver LCD Driver
  * @{
  */

/** @defgroup __1986BE9x_LCD 1986BE92Y and 1986BE93Y evaluation boards specific
  * @{
  */

/** @defgroup __1986BE9x_LCD_Exported_Constants 1986BE9x LCD Exported Constants
  * @{
  */

/* LCD control port pins definitions */

/* LCD crystals control */
#if defined (USE_1986BE92Y_Rev0) || defined (USE_1986BE92Y_Rev1)
#define LCD_CRYSTAL_PINs                 (PORT_Pin_7 | PORT_Pin_8)
#define LCD_CRYSTAL_POS                  7
#define LCD_CRYSTAL_PORT                 PORTB
#elif defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)
#define LCD_CRYSTAL_PINs                 (PORT_Pin_0 | PORT_Pin_1)
#define LCD_CRYSTAL_POS                  0
#define LCD_CRYSTAL_PORT                 PORTF
#endif

/* LCD command/data switching */
#if defined (USE_1986BE92Y_Rev0) || defined (USE_1986BE92Y_Rev1)
#define LCD_CMD_DATA_PIN                 (PORT_Pin_0)
#define LCD_CMD_DATA_PORT                PORTC
#elif defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)
#define LCD_CMD_DATA_PIN                 (PORT_Pin_4)
#define LCD_CMD_DATA_PORT                PORTF
#endif

/* LCD read/write switching */
#if defined (USE_1986BE92Y_Rev0) || defined (USE_1986BE92Y_Rev1)
#define LCD_RD_WR_PIN                    (PORT_Pin_10)
#define LCD_RD_WR_PORT                   PORTB
#elif defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)
#define LCD_RD_WR_PIN                    (PORT_Pin_3)
#define LCD_RD_WR_PORT                   PORTF
#endif

/* LCD CLOCK signal control */
#if defined (USE_1986BE92Y_Rev0) || defined (USE_1986BE92Y_Rev1)
#define LCD_CLOCK_PIN                    (PORT_Pin_1)
#define LCD_CLOCK_PORT                   PORTC
#elif defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)
#define LCD_CLOCK_PIN                    (PORT_Pin_3)
#define LCD_CLOCK_PORT                   PORTD
#endif

/* LCD RESET signal control */
#if defined (USE_1986BE92Y_Rev0) || defined (USE_1986BE92Y_Rev1)
#define LCD_RESET_PIN                    (PORT_Pin_9)
#define LCD_RESET_PORT                   PORTB
#elif defined (USE_1986BE93Y_Rev0) || defined (USE_1986BE93Y_Rev1)
#define LCD_RESET_PIN                    (PORT_Pin_2)
#define LCD_RESET_PORT                   PORTF
#endif

/** @} */ /* End of group __1986BE9x_LCD_Exported_Constants */

/** @} */ /* End of group __1986BE9x_LCD */

/** @} */ /* End of group LCD_Driver */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

#endif /* __LCD_H */

#endif /* USE_1986BE92Y_Rev0  || USE_1986BE92Y_Rev1 ||
        * USE_1986BE93Y_Rev0  || USE_1986BE93Y_Rev1 */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE lcd_1986BE9x.h */

