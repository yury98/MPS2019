/**
  ******************************************************************************
  * @file    Demo_Init_1986BE91.c
  * @author  Phyton Application Team
  * @version V2.0.0
  * @date    09.09.2010
  * @brief   Initializes the demonstration application for 1986BE91_Rev0 and
  *          1986BE91_Rev1 evaluation boards
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

/* Includes ------------------------------------------------------------------*/
#include "1986be9x_config.h"
#include <1986be9x.h>
#include <1986be9x_port.h>
#include <1986be9x_rst_clk.h>
#include <1986be9x_uart.h>
#include <1986BE9x_ext_bus.h>
#include "1986be9x_board.h"
#include "lcd.h"
#include "leds.h"
#include "demo_init.h"

#if defined (USE_1986BE91_Rev0) || defined (USE_1986BE91_Rev1)

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @addtogroup Demo_Setup Demo Setup
  * @{
  */

/** @defgroup __1986BE91_Demo_Setup 1986BE91 evaluation boards specific
  * @{
  */

/** @defgroup __1986BE91_Demo_Setup_Private_Constants 1986BE91 Demo Setup Private Constants
  * @{
  */

#define ALL_PORTS_CLK   (RST_CLK_PCLK_PORTA | RST_CLK_PCLK_PORTB | \
                         RST_CLK_PCLK_PORTC | RST_CLK_PCLK_PORTD | \
                         RST_CLK_PCLK_PORTE | RST_CLK_PCLK_PORTF)

/** @} */ /* End of group __1986BE91_Demo_Setup_Private_Constants */

/** @defgroup __1986BE91_Demo_Setup_Private_Variables 1986BE91 Demo Setup Private Variables
  * @{
  */

/** @} */ /* End of group __1986BE91_Demo_Setup_Private_Variables */

/** @defgroup __1986BE91_Demo_Setup_Private_Functions 1986BE91 Demo Setup Private Functions
  * @{
  */

/*******************************************************************************
* Function Name  : ExtBus_Setup
* Description    : Configures the External Bus.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ExtBus_Setup(void)
{
	EXT_BUS_InitTypeDef EXT_BUS_InitStruct;

	/* Enables the RTCHSE clock on the EXT_BUS */
  RST_CLK_PCLKcmd(RST_CLK_PCLK_EXT_BUS_CNTRL, ENABLE);

  EXT_BUS_StructInit(&EXT_BUS_InitStruct);

  /* Configure External Bus */
  EXT_BUS_InitStruct.EXT_BUS_WaitState = EXT_BUS_WAIT_STATE_12HCLK;
  EXT_BUS_InitStruct.EXT_BUS_Mode      = EXT_BUS_MODE_ROM;

  EXT_BUS_Init(&EXT_BUS_InitStruct);
}

/*******************************************************************************
* Function Name  : ClockConfigure
* Description    : Configures the CPU_PLL and RTCHSE clock.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ClockConfigure(void)
{
  /* Configure CPU_PLL clock */
  RST_CLK_CPU_PLLconfig (RST_CLK_CPU_PLLsrcHSIdiv2,0);

  /* Enables the RTCHSE clock on all ports */
  RST_CLK_PCLKcmd(ALL_PORTS_CLK, ENABLE);
}

/*******************************************************************************
* Function Name  : Demo_Init
* Description    : Initializes the demonstration application
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Demo_Init(void)
{
  ClockConfigure();

/************************ LCD Initialization *************************/

  /* Configure PORTA pins for data transfer to/from LCD */
  PortInitStructure.PORT_Pin = LCD_DATA_BUS_8;
  PortInitStructure.PORT_FUNC = PORT_FUNC_MAIN;
  PortInitStructure.PORT_SPEED = PORT_SPEED_SLOW;
  PortInitStructure.PORT_MODE = PORT_MODE_DIGITAL;
  PortInitStructure.PORT_OE = PORT_OE_IN;

  PORT_Init(PORTA, &PortInitStructure);

  /* Configure PORTE pin4 and pin5 for LCD crystals control */
  PortInitStructure.PORT_Pin = PORT_Pin_4 | PORT_Pin_5;

  PORT_Init(PORTE, &PortInitStructure);

  /* Configure PORTE pin11 for data/command mode switching */
  PortInitStructure.PORT_Pin = PORT_Pin_11;

  PORT_Init(PORTE, &PortInitStructure);

  /* Configure PORTC pin2 for read/write control */
  PortInitStructure.PORT_Pin = PORT_Pin_2;

  PORT_Init(PORTC, &PortInitStructure);

  /* Configure PORTC pin7 for CLOCK signal control */
  PortInitStructure.PORT_Pin = PORT_Pin_7;
  PortInitStructure.PORT_SPEED = PORT_SPEED_FAST;

  PORT_Init(PORTC, &PortInitStructure);

  /* Configure PORTC pin9 for RESET signal control */
  PortInitStructure.PORT_Pin = PORT_Pin_9;
  PortInitStructure.PORT_OE = PORT_OE_OUT;
  PortInitStructure.PORT_FUNC = PORT_FUNC_PORT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(PORTC, &PortInitStructure);

  ExtBus_Setup();

  LCD_INIT();

/************************ Joystick Initialization *************************/

  /* Configure PORTC pins 10..14 for input to handle joystick events */
  PortInitStructure.PORT_Pin   = (PORT_Pin_10 | PORT_Pin_11 | PORT_Pin_12 |
                                  PORT_Pin_13 | PORT_Pin_14);
  PortInitStructure.PORT_OE    = PORT_OE_IN;
  PortInitStructure.PORT_FUNC  = PORT_FUNC_PORT;
  PortInitStructure.PORT_SPEED = PORT_OUTPUT_OFF;

  PORT_Init(PORTC, &PortInitStructure);

/************************ LEDs Initialization *************************/

  /* Configure PORTD pins 10..14 for output to switch LEDs on/off */
  PortInitStructure.PORT_Pin   = LEDs_PINs;
  PortInitStructure.PORT_OE    = PORT_OE_OUT;
  PortInitStructure.PORT_FUNC  = PORT_FUNC_PORT;
  PortInitStructure.PORT_SPEED = PORT_SPEED_SLOW;

  PORT_Init(PORTD, &PortInitStructure);

  /* All LEDs switch off */
  PORT_ResetBits(PORTD, LEDs_PINs);
}

/** @} */ /* End of group __1986BE91_Demo_Setup_Private_Functions */

/** @} */ /* End of group __1986BE91_Demo_Setup */

/** @} */ /* End of group Demo_Setup */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

#endif /* USE_1986BE91_Rev0 || USE_1986BE91_Rev1 */

/******************* (C) COPYRIGHT 2010 Phyton *******************
*
* END OF FILE Demo_Init_1986BE91.c */

