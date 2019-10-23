/**
  ******************************************************************************
  * @file    lcd_1986BE91.h
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    10.09.2010
  * @brief   This file contains all the specific types, constants and variables
  *          for the LCD driver for 1986BE91_Rev0 and 1986BE91_Rev1 evaluation
  *          boards.
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

#if defined (USE_1986BE91_Rev0) || defined (USE_1986BE91_Rev1)

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LCD_1986BE91_H
#define __LCD_1986BE91_H

/* Includes ------------------------------------------------------------------*/
#include <1986be9x.h>
#include <1986be9x_port.h>
#include <1986be9x_rst_clk.h>
#include "types.h"
#include "lcd.h"

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @addtogroup LCD_Driver LCD Driver
  * @{
  */

/** @defgroup __1986BE91_LCD 1986BE91 evaluation boards specific
  * @{
  */

/** @defgroup __1986BE91_LCD_Exported_Types 1986BE91 LCD Exported Types
  * @{
  */

/* LCD crystal ports */
typedef struct
{
    uint32_t Data;
    uint32_t Cmd;
}LCD_Ports;

/** @} */ /* End of group __1986BE91_LCD_Exported_Types */

/** @defgroup __1986BE91_LCD_Exported_Variables 1986BE91 LCD Exported Variables
  * @{
  */

extern const LCD_Ports CrystalPorts[NUM_LCD_CRYSTALS];

/** @} */ /* End of group __1986BE91_LCD_Exported_Variables */

/** @defgroup __1986BE91_LCD_Exported_Macros 1986BE91 LCD Exported Macros
  * @{
  */

#define LCD_DATA(x)                 (*((vuint32_t*)CrystalPorts[x].Data))
#define LCD_CMD(x)                  (*((vuint32_t*)CrystalPorts[x].Cmd))

/** @} */ /* End of group __1986BE91_LCD_Exported_Macros */

/** @} */ /* End of group __1986BE91_LCD */

/** @} */ /* End of group LCD_Driver */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

#endif /* __LCD_1986BE91_H */

#endif /* USE_1986BE91_Rev0 || USE_1986BE91_Rev1 */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE lcd_1986BE91.h */

