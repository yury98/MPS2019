/**
  ******************************************************************************
  * @file    1986BE9x_bkp.c
  * @author  Phyton Application Team
  * @version V1.0.0
  * @date    21/07/2011
  * @brief   This file contains all the BKP firmware functions.
  ******************************************************************************
  * <br><br>
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, PHYTON SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT
  * OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 Phyton</center></h2>
  ******************************************************************************
  * FILE 1986BE9x_bkp.c
  */
/* Includes ------------------------------------------------------------------*/
#include "1986BE9x_bkp.h"
#include "1986BE9x_rst_clk.h"
#include "1986BE9x_config.h"

#define ASSERT_INFO_FILE_ID FILEID__1986BE9X_BKP_C

/** @addtogroup __1986BE9x_StdPeriph_Driver 1986BE9x Standard Peripherial Driver
  * @{
  */

/** @defgroup BKP
  * @brief BKP driver modules
  * @{
  */

/** @defgroup BKP_Private_TypesDefinitions BKP Private Types Definitions
  * @{
  */

/** @} */ /* End of group BKP_Private_TypesDefinitions */

/** @defgroup BKP_Private_Defines BKP Private Defines
  * @{
  */

/* BKP registers bit address in the alias region */
#define PERIPH_BASE                 0x40000000
#define PERIPH_BB_BASE              0x42000000
#define BKP_OFFSET                 (BKP_BASE - PERIPH_BASE)

#define SFR_OFFSET(TP, MOD, SFR)    ((uint32_t)&((TP*)MOD)->SFR)
#define BB_ADDR(TP, MOD, SFR, BIT)  (PERIPH_BB_BASE + SFR_OFFSET(TP, MOD, SFR)*32 + BIT*4)
#define BKP_BB(SFR, BIT)            BB_ADDR(BKP_TypeDef, BKP_OFFSET, SFR, BIT)

#define BKP_JTAGA_BB                BKP_BB(REG_0E, BKP_REG_0E_JTAGA_Pos)
#define BKP_JTAGB_BB                BKP_BB(REG_0E, BKP_REG_0E_JTAGB_Pos)
#define BKP_FPOR_BB                 BKP_BB(REG_0E, BKP_REG_0E_FPOR_Pos)
#define BKP_STANDBY_BB              BKP_BB(REG_0F, BKP_REG_0F_STANDBY_Pos)
#define RTC_ENABLE_BB               BKP_BB(REG_0F, BKP_REG_0F_RTC_EN_Pos)
#define RTC_RESET_BB                BKP_BB(REG_0F, BKP_REG_0F_RTC_RESET_Pos)
#define RTC_WEC_BB                  BKP_BB(RTC_CS, BKP_RTC_CS_WEC_Pos)

/* --------------------- BKP registers bit mask ------------------------ */

/* BKP_REG0E register bit mask */
#define DUcc_Mask         ((uint32_t)(BKP_REG_0E_LOW_Msk | BKP_REG_0E_SelectRI_Msk))
#define DUccTrim_Mask     ((uint32_t)BKP_REG_0E_Trim_Msk)
#define BKP_REG_0E_ON     ((uint32_t)(0x01U<<11))
#define BKP_REG_0E_OFF    ((uint32_t)(~((0x01U<<31)|(0x07U<<8)|(0x3FU))))

/* CFR register bit mask */
#define WDGTB_Mask        ((uint32_t)0xFFFFFE7F)
#define WIN_Mask          ((uint32_t)0x7F)
#define CTR_Mask          ((uint32_t)0x7F)

/** @} */ /* End of group BKP_Private_Defines */

/** @defgroup BKP_Private_Macros BKP Private Macros
  * @{
  */

/** @} */ /* End of group BKP_Private_Macros */

/** @defgroup BKP_Private_Variables BKP Private Variables
  * @{
  */

/** @} */ /* End of group BKP_Private_Variables */

/** @defgroup BKP_Private_Functions BKP Private Functions
  * @{
  */

/**
  * @brief  BKP_DeInit - Deinitializes the BKP peripheral registers to their default reset values.
  * @param  None
  * @retval None
  */
void BKP_DeInit(void)
{
  #define BKP_RTC_CS_NUM ((uint32_t) (SFR_OFFSET(BKP_TypeDef, BKP_OFFSET, RTC_CS) - BKP_OFFSET)/4)
  #define BKP_REG_0F_NUM ((uint32_t) (SFR_OFFSET(BKP_TypeDef, BKP_OFFSET, REG_0F) - BKP_OFFSET)/4)
  #define BKP_REG_0E_NUM ((uint32_t) (SFR_OFFSET(BKP_TypeDef, BKP_OFFSET, REG_0E) - BKP_OFFSET)/4)
  #define BKP_RTC_CNT_NUM ((uint32_t) (SFR_OFFSET(BKP_TypeDef, BKP_OFFSET, RTC_CNT) - BKP_OFFSET)/4)

  uint32_t * _bkp = (uint32_t *) BKP_BASE;
  uint32_t i;

  for ( i = 0; i < BKP_REG_0E_NUM; i++)
  {
    _bkp[i] = 0;
  }

  for ( i = BKP_RTC_CNT_NUM; i <= BKP_RTC_CS_NUM; i++)
  {
    _bkp[i] = 0;
  }

  BKP -> REG_0E |= (uint32_t) (BKP_REG_0E_ON);
  BKP -> REG_0E &= (uint32_t) (BKP_REG_0E_OFF);
  BKP -> REG_0F = (uint32_t) (BKP_REG_0F_LSI_ON);
}

/**
  * @brief  Enables or disables the JTAG A.
  * @param  NewState: new state of the JTAG A.
  *         @arg ENABLE
  *         @arg DISABLE
  * @retval None
  */
void BKP_JTAGA_Enable(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(__IO uint32_t *) BKP_JTAGA_BB = (uint32_t)NewState;
}

/**
  * @brief  Enables or disables the JTAG B.
  * @param  NewState: new state of the JTAG B.
  *         @arg ENABLE
  *         @arg DISABLE
  * @retval None
  */
void BKP_JTAGB_Enable(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(__IO uint32_t *) BKP_STANDBY_BB = (uint32_t)NewState;
}

/**
  * @brief  BKP_RTCclkSource - Configures the RTC clock source.
  * @param  RTC_CLK: specifies the RTC clock source.
  *         @arg BKP_RTC_LSIclk: LSI oscillator clock is selected as RTC clock source.
  *         @arg BKP_RTC_LSEclk: LSE oscillator clock is selected as RTC clock source.
  *         @arg BKP_RTC_HSIclk: HSI oscillator clock is selected as RTC clock source.
  *         @arg BKP_RTC_HSEclk: HSE oscillator clock is selected as RTC clock source.
  * @retval None
  */
void BKP_RTCclkSource(uint32_t RTC_CLK)
{
  uint32_t tmpreg;

  /* Check the parameters */
  assert_param(IS_RTC_CLK_SOURCE(RTC_CLK));
  /* Clear BKP_REG0F[3:2] bits */
  tmpreg  = BKP -> REG_0F & (uint32_t) (~BKP_REG_0F_RTC_SEL_Msk);
  /* Set BKP_REG0F[3:2] bits according to RTC clock source*/
  tmpreg |= BKP_REG_0F_RTC_SEL_Msk & RTC_CLK;
  BKP -> REG_0F = tmpreg;
}


/**
  * @brief  BKP_RTC_Enable - Enables or disables the RTC.
  * @param  NewState: new state of the RTC.
  *         @arg ENABLE
  *         @arg DISABLE
  * @retval None
  */
void BKP_RTC_Enable(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(__IO uint32_t *) RTC_ENABLE_BB = (uint32_t)NewState;
}

/**
  * @brief  BKP_RTC_Calibration - Configures the RTC clock source.
  * @param  RTC_Calibration: specifies the tact drop count.
  *         @arg 0 <= RTC_Calibration <= 255.
  * @retval None
  */
void BKP_RTC_Calibration(uint32_t RTC_Calibration)
{
  uint32_t tmpreg;

  /* Check the parameters */
  assert_param(IS_RTC_CALIBRATION(RTC_Calibration));
  /* Clear BKP_REG0F[12:5] bits */
  tmpreg  = BKP -> REG_0F & (uint32_t) (~BKP_REG_0F_CAL_Msk);
  /* Set BKP_REG0F[12:5] bits according to RTC clock source*/
  tmpreg |= BKP_REG_0F_CAL_Msk & (RTC_Calibration << BKP_REG_0F_CAL_Pos);
  BKP -> REG_0F = tmpreg;
}


/**
  * @brief  BKP_RTC_Reset -  Reset the RTC.
  * @param  NewState: new state of the RTC.
  *         @arg ENABLE  : Reset the RTC
  *         @arg DISABLE : Not reset the RTC
  * @retval None
  */
void BKP_RTC_Reset(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  *(__IO uint32_t *) RTC_RESET_BB = (uint32_t)NewState;
}

/**
  * @brief  BKP_RTC_ITConfig - Enables or disables the specified RTC interrupts.
  * @param  BKP_RTC_IT: specifies the RTC interrupts sources to be enabled or disabled.
  *   This parameter can be any combination of the following values:
  *     @arg BKP_RTC_IT_ALRF: Alarm interrupt enable
  *     @arg BKP_RTC_IT_SECF: Second interrupt enable
  *     @arg BKP_RTC_IT_OWF : Overflow interrupt enable
  * @param  NewState: new state of the specified RTC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void BKP_RTC_ITConfig(uint32_t BKP_RTC_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_RTC_CLK_IT(BKP_RTC_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    BKP->RTC_CS |= BKP_RTC_IT;
  }
  else
  {
    BKP->RTC_CS &= (uint32_t)~BKP_RTC_IT;
  }
}

/**
  * @brief  BKP_RTC_GetCounter - Gets the RTC counter value.
  * @param  None
  * @retval RTC counter value.
  */
uint32_t BKP_RTC_GetCounter(void)
{
  return BKP -> RTC_CNT;
}

/**
  * @brief  BKP_RTC_SetCounter - Sets the RTC counter value.
  * @param  CounterValue: RTC counter new value.
  * @retval None
  */
void BKP_RTC_SetCounter(uint32_t CounterValue)
{
  BKP -> RTC_CNT = CounterValue;
}

/**
  * @brief  BKP_RTC_SetAlarm - Sets the RTC alarm value.
  * @param  AlarmValue: RTC alarm new value.
  * @retval None
  */
void BKP_RTC_SetAlarm(uint32_t AlarmValue)
{
  BKP -> RTC_ALRM = AlarmValue;
}

/**
  * @brief  BKP_RTC_GetDivider - Gets the RTC divider value.
  * @param  None
  * @retval RTC Divider value.
  */
uint32_t BKP_RTC_GetDivider(void)
{
  return BKP -> RTC_DIV;
}

/**
  * @brief  BKP_RTC_SetPrescaler - Sets the RTC prescaler (RTC_DIV) preloaded value.
  * @param  PrescalerValue: RTC prescaler new value.
  *         This parameter should be less or equal then 0x000FFFFF
  * @retval None
  */
void BKP_RTC_SetPrescaler(uint32_t PrescalerValue)
{
  /* Check the parameter */
  assert_param(IS_BKP_PRESCALER_VALUE(PrescalerValue));

  BKP -> RTC_PRL = PrescalerValue;
}

/**
  * @brief  BKP_RTC_WaitForUpdate - Waits until last write operation on RTC registers has finished.
  * @note   This function must be called before any write to RTC registers.
  * @param  None
  * @retval None
  */
void BKP_RTC_WaitForUpdate(void)
{
  /* Loop until WEC flag is set */
  while (*(__IO uint32_t *) RTC_WEC_BB != 0)
  {
  }
}

/**
  * @brief  BKP_RTC_GetFlagStatus - Checks whether the specified RTC flag is set or not.
  * @param  BKP_RTC_FLAG: specifies the flag to check.
  *   This parameter can be one the following values:
  *     @arg BKP_RTC_FLAG_WEC: RTC Write operation ended flag
  *     @arg BKP_RTC_FLAG_OWF: Overflow flag
  *     @arg BKP_RTC_FLAG_ALRF: Alarm flag
  *     @arg BKP_RTC_FLAG_SECF: Second flag
  * @retval The new state of BKP_RTC_FLAG (SET or RESET).
  */
FlagStatus BKP_RTC_GetFlagStatus(uint32_t BKP_RTC_FLAG)
{
  FlagStatus status;

  /* Check the parameters */
  assert_param(IS_BKP_RTC_GET_FLAG(BKP_RTC_FLAG));

  if ((BKP -> RTC_CS & BKP_RTC_FLAG) != 0)
  {
    status = SET;
  }
  else
  {
    status = RESET;
  }
  return status;
}

/** @} */ /* End of group BKP_Private_Functions */

/** @} */ /* End of group BKP */

/** @} */ /* End of group __1986BE9x_StdPeriph_Driver */

/******************* (C) COPYRIGHT 2010 Phyton *********************************
*
* END OF FILE 1986BE9x_bkp.c */

