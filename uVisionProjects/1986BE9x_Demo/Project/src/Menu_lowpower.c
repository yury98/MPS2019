/**
  ******************************************************************************
  * @file    Menu_lowpower.c
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    05.12.2011
  * @brief   This file contains all the "Low Power" menu handlers.
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
#include <1986be9x_power.h>
#include <1986be9x_rst_clk.h>
#include <1986be9x_bkp.h>
#include "systick.h"
#include "Menu.h"
#include "Menu_items.h"
#include "leds.h"
#include "lcd.h"
#include "text.h"
#include "joystick.h"
#include "time.h"
#include "demo_init.h"
#include "1986BE9x_it.h"

/** @addtogroup __1986BE9x_Eval_Demo 1986BE9x Demonstration Example
  * @{
  */

/** @addtogroup Menu Menu
  * @{
  */

/** @addtogroup Menu_LowPower Menu Low Power
  * @{
  */

/** @defgroup Menu_LowPower_Private_Constants Menu Low Power Private Constants
  * @{
  */

/** @} */ /* End of group Menu_LowPower_Private_Constants */

/** @defgroup Menu_LowPower_Private_Types Menu Low Power Private Types
  * @{
  */

typedef struct
{
  uint32_t OE;
  uint32_t FUNC;
  uint32_t ANALOG;
  uint32_t PULL;
  uint32_t PD;
  uint32_t PWR;
  uint32_t GFEN;
}PORTConfigData;

typedef struct
{
  uint32_t PVDCS;
  uint32_t CPU_CLOCK;
  uint32_t PER_CLOCK;
#if defined (USE_1986BE91_Rev0) || defined (USE_1986BE91_Rev1)
  uint32_t EXT_BUS_CONTROL;
#endif
}ClockConfigData;

/** @} */ /* End of group Menu_LowPower_Private_Types */

/** @defgroup Menu_LowPower_Private_Macros Menu Low Power Private Macros
  * @{
  */

/** @} */ /* End of group Menu_LowPower_Private_Macros */

/** @defgroup Menu_LowPower_Private_Variables Menu Low Power Private Variables
  * @{
  */
static PORTConfigData PORTA_Data, PORTB_Data, PORTC_Data, PORTD_Data, PORTE_Data,
                      PORTF_Data;

static ClockConfigData ClockData;

/** @} */ /* End of group Menu_LowPower_Private_Variables */

/** @defgroup Menu_LowPower_Private_Functions Menu Low Power Private Functions
  * @{
  */

/*******************************************************************************
* Function Name  : LowPower_Init
* Description    : Initializes Low Power application.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LowPower_Init(void)
{
  RST_CLK_PCLKcmd(RST_CLK_PCLK_POWER, ENABLE);
}

/*******************************************************************************
* Function Name  : PORT_SaveConfig
* Description    : Save all PORTs Configurations.
* Input          : None
* Output         : PORTA_Data, PORTB_Data, PORTC_Data, PORTD_Data, PORTE_Data,
*                : PORTF_Data.
* Return         : None
*******************************************************************************/
static void PORT_SaveConfig(void)
{
  PORTA_Data.ANALOG = PORTA->ANALOG;
  PORTA_Data.FUNC   = PORTA->FUNC;
  PORTA_Data.GFEN   = PORTA->GFEN;
  PORTA_Data.OE     = PORTA->OE;
  PORTA_Data.PD     = PORTA->PD;
  PORTA_Data.PULL   = PORTA->PULL;
  PORTA_Data.PWR    = PORTA->PWR;

  PORTB_Data.ANALOG = PORTB->ANALOG;
  PORTB_Data.FUNC   = PORTB->FUNC;
  PORTB_Data.GFEN   = PORTB->GFEN;
  PORTB_Data.OE     = PORTB->OE;
  PORTB_Data.PD     = PORTB->PD;
  PORTB_Data.PULL   = PORTB->PULL;
  PORTB_Data.PWR    = PORTB->PWR;

  PORTC_Data.ANALOG = PORTC->ANALOG;
  PORTC_Data.FUNC   = PORTC->FUNC;
  PORTC_Data.GFEN   = PORTC->GFEN;
  PORTC_Data.OE     = PORTC->OE;
  PORTC_Data.PD     = PORTC->PD;
  PORTC_Data.PULL   = PORTC->PULL;
  PORTC_Data.PWR    = PORTC->PWR;

  PORTD_Data.ANALOG = PORTD->ANALOG;
  PORTD_Data.FUNC   = PORTD->FUNC;
  PORTD_Data.GFEN   = PORTD->GFEN;
  PORTD_Data.OE     = PORTD->OE;
  PORTD_Data.PD     = PORTD->PD;
  PORTD_Data.PULL   = PORTD->PULL;
  PORTD_Data.PWR    = PORTD->PWR;

  PORTE_Data.ANALOG = PORTE->ANALOG;
  PORTE_Data.FUNC   = PORTE->FUNC;
  PORTE_Data.GFEN   = PORTE->GFEN;
  PORTE_Data.OE     = PORTE->OE;
  PORTE_Data.PD     = PORTE->PD;
  PORTE_Data.PULL   = PORTE->PULL;
  PORTE_Data.PWR    = PORTE->PWR;

  PORTF_Data.ANALOG = PORTF->ANALOG;
  PORTF_Data.FUNC   = PORTF->FUNC;
  PORTF_Data.GFEN   = PORTF->GFEN;
  PORTF_Data.OE     = PORTF->OE;
  PORTF_Data.PD     = PORTF->PD;
  PORTF_Data.PULL   = PORTF->PULL;
  PORTF_Data.PWR    = PORTF->PWR;

  /* Configure all PORTs for low power consumption */
  PORT_DeInit(PORTA);
  PORT_DeInit(PORTB);
  PORT_DeInit(PORTC);
  PORT_DeInit(PORTD);
  PORT_DeInit(PORTE);
  PORT_DeInit(PORTF);
}

/*******************************************************************************
* Function Name  : PORT_RestoreConfig
* Description    : Restores all PORTs Configurations.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void PORT_RestoreConfig(void)
{
  PORTA->ANALOG = PORTA_Data.ANALOG;
  PORTA->FUNC   = PORTA_Data.FUNC;
  PORTA->GFEN   = PORTA_Data.GFEN;
  PORTA->OE     = PORTA_Data.OE;
  PORTA->PD     = PORTA_Data.PD;
  PORTA->PULL   = PORTA_Data.PULL;
  PORTA->PWR    = PORTA_Data.PWR;

  PORTB->ANALOG = PORTB_Data.ANALOG;
  PORTB->FUNC   = PORTB_Data.FUNC;
  PORTB->GFEN   = PORTB_Data.GFEN;
  PORTB->OE     = PORTB_Data.OE;
  PORTB->PD     = PORTB_Data.PD;
  PORTB->PULL   = PORTB_Data.PULL;
  PORTB->PWR    = PORTB_Data.PWR;

  PORTC->ANALOG = PORTC_Data.ANALOG;
  PORTC->FUNC   = PORTC_Data.FUNC;
  PORTC->GFEN   = PORTC_Data.GFEN;
  PORTC->OE     = PORTC_Data.OE;
  PORTC->PD     = PORTC_Data.PD;
  PORTC->PULL   = PORTC_Data.PULL;
  PORTC->PWR    = PORTC_Data.PWR;

  PORTD->ANALOG = PORTD_Data.ANALOG;
  PORTD->FUNC   = PORTD_Data.FUNC;
  PORTD->GFEN   = PORTD_Data.GFEN;
  PORTD->OE     = PORTD_Data.OE;
  PORTD->PD     = PORTD_Data.PD;
  PORTD->PULL   = PORTD_Data.PULL;
  PORTD->PWR    = PORTD_Data.PWR;

  PORTE->ANALOG = PORTE_Data.ANALOG;
  PORTE->FUNC   = PORTE_Data.FUNC;
  PORTE->GFEN   = PORTE_Data.GFEN;
  PORTE->OE     = PORTE_Data.OE;
  PORTE->PD     = PORTE_Data.PD;
  PORTE->PULL   = PORTE_Data.PULL;
  PORTE->PWR    = PORTE_Data.PWR;

  PORTF->ANALOG = PORTF_Data.ANALOG;
  PORTF->FUNC   = PORTF_Data.FUNC;
  PORTF->GFEN   = PORTF_Data.GFEN;
  PORTF->OE     = PORTF_Data.OE;
  PORTF->PD     = PORTF_Data.PD;
  PORTF->PULL   = PORTF_Data.PULL;
  PORTF->PWR    = PORTF_Data.PWR;
}

/*******************************************************************************
* Function Name  : Clock_SaveConfig
* Description    : Save the Power and Clock configuration.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Clock_SaveConfig(void)
{
  ClockData.PVDCS           = POWER->PVDCS;
  ClockData.CPU_CLOCK       = RST_CLK->CPU_CLOCK;
  ClockData.PER_CLOCK       = RST_CLK->PER_CLOCK;
#if defined (USE_1986BE91_Rev0) || defined (USE_1986BE91_Rev1)
  ClockData.EXT_BUS_CONTROL = EXT_BUS->CONTROL;
#endif
}

/*******************************************************************************
* Function Name  : Clock_RestoreConfig
* Description    : Restore the Power and Clock configuration.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Clock_RestoreConfig(void)
{
  POWER->PVDCS       = ClockData.PVDCS;
  RST_CLK->CPU_CLOCK = ClockData.CPU_CLOCK;
  RST_CLK->PER_CLOCK = ClockData.PER_CLOCK;
#if defined (USE_1986BE91_Rev0) || defined (USE_1986BE91_Rev1)
  EXT_BUS->CONTROL   = ClockData.EXT_BUS_CONTROL;
#endif
}

/*******************************************************************************
* Function Name  : STANDBYMode_WAKEUP
* Description    : Enters MCU in STANDBY mode.
*                : Attention: the wake-up from STANDBY mode is performed on
*                : WAKEUP pin low level. After STANDBY mode exiting, power on
*                : reset is occurred.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STANDBYMode_WAKEUP(void)
{
  /* Print the header */
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("STANDBY. Exit: WAKEUP");
  WAIT_UNTIL_KEY_RELEASED(SEL);

  LCD_PUTS(0, LineMessage1, "                    ");
  LCD_PUTS(0, LineMessage2, "                    ");
  LCD_PUTS(0, LineMessage3, "MCU in STANDBY Mode ");
  LCD_PUTS(0, LineMessage4, "To exit press Wakeup");
  LCD_PUTS(0, LineMessage5, "                     ");

  /* Request to enter STANDBY mode */
  POWER_EnterSTANDBYMode();
}

/*******************************************************************************
* Function Name  : STANDBYMode_RTCAlarm
* Description    : Enters MCU in STANDBY mode.
*                : Attention: the wake-up from STANDBY mode is performed on
*                : RTC Alarm event. After STANDBY mode exiting, power on
*                : reset is occurred.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STANDBYMode_RTCAlarm(void)
{
  /* Print the header */
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("STANDBY. Exit: Alarm");
  WAIT_UNTIL_KEY_RELEASED(SEL);

  if((BKP->REG_00 != 0x1234) || (BKP->RTC_PRL != RTC_PRESCALER_VALUE))
  {
    LCD_PUTS(0, LineMessage1, "RTC is not configured");
    LCD_PUTS(0, LineMessage2, "Please, use the ");
    LCD_PUTS(0, LineMessage3, "Time Adjust menu and ");
    LCD_PUTS(0, LineMessage4, "Alarm Adjust menu to ");
    LCD_PUTS(0, LineMessage5, "set the Alarm time. ");
    WAIT_UNTIL_KEY_PRESSED(SEL);
    WAIT_UNTIL_KEY_RELEASED(SEL);

    /* Display the menu */
    DisplayMenu();
    return;
  }

  LCD_PUTS(0, LineMessage1, "                    ");
  LCD_PUTS(0, LineMessage2, "                    ");
  LCD_PUTS(0, LineMessage3, " MCU in STANDBY Mode");
  LCD_PUTS(0, LineMessage4, " Wait For RTC Alarm ");
  LCD_PUTS(0, LineMessage5, "                     ");

  /* Request to enter STANDBY mode */
  POWER_EnterSTANDBYMode();
}

/*******************************************************************************
* Function Name  : STOPMode_RTCAlarm
* Description    : Enters MCU in STOP mode. The wake-up from STOP mode is
*                  performed by RTC Alarm.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STOPMode_RTCAlarm(void)
{
  uint32_t tmp;

  /* Print the header */
  LCD_CLS();
  CurrentMethod = MET_AND;
  CurrentFont = &Font_6x8;
  DisplayMenuTitle("STOP. Exit: Alarm");
  WAIT_UNTIL_KEY_RELEASED(SEL);

  if((BKP->REG_00 != 0x1234) || (BKP->RTC_PRL != RTC_PRESCALER_VALUE))
  {
    LCD_PUTS(0, LineMessage1, "RTC is not configured");
    LCD_PUTS(0, LineMessage2, "Please, use the ");
    LCD_PUTS(0, LineMessage3, "Time Adjust menu and ");
    LCD_PUTS(0, LineMessage4, "Alarm Adjust menu to ");
    LCD_PUTS(0, LineMessage5, "set the Alarm time.");
    WAIT_UNTIL_KEY_PRESSED(SEL);
    WAIT_UNTIL_KEY_RELEASED(SEL);

    /* Display the menu */
    DisplayMenu();
    return;
  }

  STOPModeStatus = 1;

  LCD_PUTS(0, LineMessage1, "                     ");
  LCD_PUTS(0, LineMessage2, "                     ");
  LCD_PUTS(0, LineMessage3, "  MCU in STOP Mode   ");
  LCD_PUTS(0, LineMessage4, " Wait For RTC Alarm  ");
  LCD_PUTS(0, LineMessage5, "                     ");

  /* Save the all PORTs current configuration, then
   * configure all PORTs for low power consumption */
  PORT_SaveConfig();

  /* Save the Power and Clock current configuration */
  Clock_SaveConfig();

  /* Enable SLEEPONEXIT mode */
  tmp = SCB->SCR;
  tmp |= SCB_SCR_SLEEPONEXIT_Msk;
  SCB->SCR = tmp;

  /* Request to enter STOP mode with regulator ON */
  POWER_EnterSTOPMode(ENABLE, POWER_STOPentry_WFI);

  /* Restore the Power and Clock */
  Clock_RestoreConfig();

  /* Restore the PORTs Configurations*/
  PORT_RestoreConfig();

  LCD_PUTS(0, LineMessage3, "Wake-Up by RTC Alarm");
  LCD_PUTS(0, LineMessage4, "Press SEL to continue");

  /* Wait for SEL to continue */
  WAIT_UNTIL_KEY_PRESSED(SEL);
  WAIT_UNTIL_KEY_RELEASED(SEL);

  /* Display the previous menu */
  DisplayMenu();
}

/** @} */ /* End of group Menu_LowPower_Private_Functions */

/** @} */ /* End of group Menu_LowPower */

/** @} */ /* End of group Menu */

/** @} */ /* End of group __1986BE9x_Eval_Demo */

/******************* (C) COPYRIGHT 2010 Phyton *******************
*
* END OF FILE Menu_lowpower.c */

